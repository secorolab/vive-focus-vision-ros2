"""Read-only, timestamped episode capture. ROS and LeRobot live in separate processes."""
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import json
import math
import os
import queue
import signal
import shutil
import subprocess
import threading
import time
import uuid


@dataclass(frozen=True)
class Sample:
    at: float
    values: tuple


class Signals:
    """Small timestamp buffers; callbacks never perform filesystem work."""
    def __init__(self):
        self.lock = threading.Lock()
        self.buffers = {key: deque(maxlen=100) for key in ('state', 'arm', 'gripper')}
        self.health = (0.0, False)
        self.world_lock = (0.0, False)
        self.controllers = (0.0, False)
        self.tracking = 0.0

    def add(self, key, at, values):
        if not all(math.isfinite(v) for v in values):
            return
        with self.lock:
            self.buffers[key].append(Sample(at, tuple(values)))

    def sample(self, at, now):
        with self.lock:
            if now - self.health[0] > .5 or not self.health[1]:
                raise ValueError('Right driver health missing or faulted')
            if now - self.world_lock[0] > .3 or not self.world_lock[1]:
                raise ValueError('World must be locked with fresh controller input')
            if now-self.controllers[0] > 1.5 or not self.controllers[1]:
                raise ValueError('Right arm/gripper controllers not confirmed active')
            if now-self.tracking > .25:
                raise ValueError('Right controller tracking unavailable')
            result = {}
            for key, values in self.buffers.items():
                if not values or now - values[-1].at > .15:
                    raise ValueError(f'{key} feedback unavailable or stale')
                best = min(values, key=lambda sample: abs(sample.at-at))
                if abs(best.at-at) > .06:
                    raise ValueError(f'{key} cannot be synchronized')
                result[key] = best
            return result


class Recorder:
    """One owner thread serializes capture, review, discard and export transitions."""
    def __init__(self, root, cameras, task, python, fps=15, max_seconds=120):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        # Prevent two services from writing to the same session.
        import fcntl
        self.lock_file = (self.root / '.recording.lock').open('a')
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.cameras, self.task, self.python = cameras, task, python
        self.fps, self.max_seconds = fps, max_seconds
        self.signals = Signals()
        self.commands = queue.Queue(maxsize=8)
        self.stop = threading.Event()
        self.state, self.message = 'idle', 'Waiting for live cameras and ROS feedback'
        self.frames, self.path, self.file = 0, None, None
        self.ready = False
        self.export_process = None
        self.export_log = None
        self.home_process = None
        self.home_log = None
        self.home_cancelled = False
        self.last_ui_seen = time.monotonic()
        self.saved = len(list((self.root / 'episodes').glob('*/episode.json')))
        self.session = dict(version=1, arm='right', cameras=['main', 'right'], fps=fps,
                            task=task, action='controller_reference_joint_positions',
                            units=['rad']*7+['m'], max_sync_error_s=.06)
        config = self.root / 'session.json'
        if config.exists() and json.loads(config.read_text()) != self.session:
            raise ValueError('Session settings differ. Choose a new --record-root directory.')
        config.write_text(json.dumps(self.session, indent=2))
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def status(self):
        self.last_ui_seen = time.monotonic()
        return dict(state=self.state, message=self.message, ready=self.ready,
                    frames=self.frames, seconds=round(self.frames/self.fps, 1),
                    saved=self.saved, task=self.task, arm='right', cameras=['main', 'right'])

    def command(self, action):
        if action not in ('start', 'stop', 'save', 'discard', 'retry_export', 'discard_home', 'cancel_home'):
            raise ValueError('Unknown recording action')
        # Carry expected state to reject stale/double button presses.
        self.commands.put_nowait((action, self.state))

    def snapshot(self, at):
        now = time.monotonic()
        signals = self.signals.sample(at, now)
        pictures = {}
        for key in ('main', 'right'):
            frames = self.cameras[key].history_snapshot()
            if not frames or now-frames[-1][0] > .15:
                raise ValueError(f'{key} camera unavailable or stale')
            frame = min(frames, key=lambda f: abs(f[0]-at))
            if abs(frame[0]-at) > .06:
                raise ValueError(f'{key} camera cannot be synchronized')
            pictures[key] = frame
        times = [s.at for s in signals.values()] + [f[0] for f in pictures.values()]
        if max(times)-min(times) > .08:
            raise ValueError('Camera / joint sample skew exceeds 80 ms')
        return signals, pictures

    def begin(self):
        self.snapshot(time.monotonic()-.10)
        if shutil.disk_usage(self.root).free < 2*1024**3:
            raise ValueError('Need at least 2 GB free disk space')
        self.path = self.root / 'pending' / uuid.uuid4().hex
        self.path.mkdir(parents=True)
        for key in ('main', 'right'): (self.path / key).mkdir()
        self.file = (self.path / 'frames.jsonl').open('w')
        self.frames = 0
        self.started = time.monotonic()
        self.next_frame = self.started
        self.last_images = {}
        self.state, self.message = 'recording', 'Recording right arm + gripper'

    def capture(self):
        now = time.monotonic()
        if now-self.next_frame > .10:
            raise ValueError('Recorder missed a sample deadline')
        at = self.next_frame-.10  # bounded alignment delay; never delays the controller
        signals, pictures = self.snapshot(at)
        for key, (stamp, jpeg) in pictures.items():
            if stamp <= self.last_images.get(key, -1):
                raise ValueError(f'{key} repeated camera frame')
            (self.path / key / f'{self.frames:06d}.jpg').write_bytes(jpeg)
            self.last_images[key] = stamp
        row = dict(index=self.frames, timestamp=self.frames/self.fps, sample_monotonic=at,
                   observation=list(signals['state'].values),
                   action=list(signals['arm'].values+signals['gripper'].values),
                   source_monotonic={**{k:s.at for k,s in signals.items()},
                                     **{k:f[0] for k,f in pictures.items()}})
        self.file.write(json.dumps(row)+'\n')
        self.file.flush()
        self.frames += 1
        self.next_frame += 1/self.fps
        if self.frames % self.fps == 0 and shutil.disk_usage(self.root).free < 1024**3:
            raise ValueError('Low disk space')
        if self.frames >= self.fps*self.max_seconds:
            self.finish('review', 'Time limit reached; review then Save or Discard')

    def finish(self, state, message):
        if self.file:
            self.file.flush()
            os.fsync(self.file.fileno())
            self.file.close()
            self.file = None
        self.state, self.message = state, message

    def export(self):
        self.export_log = (self.root / 'export.log').open('w')
        self.export_process = subprocess.Popen(
            [self.python, str(Path(__file__).with_name('openarm_export_lerobot.py')),
             str(self.root)], stdout=self.export_log, stderr=subprocess.STDOUT,
            env={**os.environ, 'HF_HUB_OFFLINE':'1', 'OMP_NUM_THREADS':'1'})
        self.state, self.message = 'saving', 'Building local LeRobot dataset; raw episode is safe'

    def discard_pending(self):
        if self.path: shutil.rmtree(self.path)
        self.path, self.frames = None, 0

    def return_home(self):
        # Preflight observes only. The existing return service does its own motor checks.
        self.signals.sample(time.monotonic()-.10, time.monotonic())
        self.finish('review', 'Discarding unsaved attempt')
        self.discard_pending()
        self.home_cancelled = False
        self.last_ui_seen = time.monotonic()
        self.home_log = (self.root / 'return_to_zero.log').open('w')
        self.home_process = subprocess.Popen(
            ['/usr/bin/python3', str(Path(__file__).with_name('openarm_return_to_zero.py')), '--execute'],
            stdout=self.home_log, stderr=subprocess.STDOUT,
            env={**os.environ, 'ROS_DOMAIN_ID':'84', 'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST'})
        self.ready = False
        self.state, self.message = 'returning', 'Returning BOTH arms to zero; grippers unchanged. A cancels.'

    def cancel_home(self):
        if self.home_process and self.home_process.poll() is None and not self.home_cancelled:
            self.home_cancelled = True
            self.home_process.send_signal(signal.SIGINT)
            self.message = 'Cancelling return; waiting for confirmation'

    def handle(self, action, expected):
        if expected != self.state: return
        if action == 'start' and self.state == 'idle': self.begin()
        elif action == 'stop' and self.state == 'recording':
            self.finish('review' if self.frames else 'fault', 'Review: B saves; hold X discards')
        elif action == 'discard' and self.state in ('review', 'fault'):
            self.discard_pending()
            self.state, self.message = 'idle', 'Discarded; ready for another attempt'
        elif action == 'save' and self.state == 'review':
            (self.path / 'episode.json').write_text(json.dumps(dict(frames=self.frames, **self.session), indent=2))
            destination = self.root / 'episodes' / self.path.name
            destination.parent.mkdir(exist_ok=True)
            self.path.rename(destination)
            self.path = None
            self.saved += 1
            self.export()
        elif action == 'discard_home' and self.state in ('idle', 'review', 'fault', 'recording', 'home_done', 'home_error'):
            self.return_home()
        elif action == 'cancel_home' and self.state == 'returning': self.cancel_home()
        elif action == 'start' and self.state == 'home_done': self.begin()
        elif action == 'discard' and self.state == 'home_error':
            self.state, self.message = 'idle', 'Return error acknowledged; check robot before recording'
        elif action == 'retry_export' and self.state == 'export_error': self.export()

    def run(self):
        try:
            while not self.stop.wait(.005):
                try:
                    try: self.handle(*self.commands.get_nowait())
                    except queue.Empty: pass
                    if self.state == 'recording' and time.monotonic() >= self.next_frame:
                        self.capture()
                    elif self.state == 'returning':
                        now = time.monotonic()
                        with self.signals.lock:
                            input_lost = now-self.signals.world_lock[0] > .5 or not self.signals.world_lock[1] or now-self.signals.tracking > .5
                        if now-self.last_ui_seen > 3 or input_lost: self.cancel_home()
                        code = self.home_process.poll()
                        if code is not None:
                            self.home_log.close()
                            self.state = 'home_done' if code == 0 else 'home_error'
                            self.message = ('At zero within tolerance. VR disarmed; enable again before recording.'
                                if code == 0 else 'Return stopped or failed; see return_to_zero.log. VR remains disarmed.')
                    elif self.state == 'idle':
                        try:
                            self.snapshot(time.monotonic()-.10)
                            self.ready, self.message = True, 'Ready: A starts recording'
                        except ValueError as exc:
                            self.ready, self.message = False, str(exc)
                    elif self.state == 'saving' and self.export_process.poll() is not None:
                        code = self.export_process.returncode
                        self.export_log.close()
                        self.state = 'idle' if code == 0 else 'export_error'
                        self.message = 'Saved locally' if code == 0 else 'Raw episode saved; export failed. See export.log; B retries.'
                except Exception as exc:
                    self.ready = False
                    if self.state == 'saving':
                        self.state, self.message = 'export_error', str(exc)
                    elif self.state == 'recording': self.finish('fault', str(exc)+'; discard this incomplete episode')
                    else: self.message = str(exc)
        finally:
            self.finish(self.state, self.message)

    def close(self):
        self.cancel_home()
        self.stop.set()
        self.thread.join(timeout=5)
        # Pending files are retained for diagnosis, never silently accepted on restart.
        if self.export_process and self.export_process.poll() is None:
            self.export_process.wait()
        if self.home_process and self.home_process.poll() is None:
            try: self.home_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.home_process.terminate()  # loss of its keepalive stops the return service
                self.home_process.wait(timeout=3)
        if self.home_log and not self.home_log.closed: self.home_log.close()
        self.lock_file.close()
