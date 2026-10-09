#!/usr/bin/env python3
"""Recorder integrity checks with synthetic feedback; never connects to a robot."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch, MagicMock
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from openarm_episode import Recorder, Signals


class Camera:
    def __init__(self): self.frames = []
    def history_snapshot(self): return self.frames[:]


def feed(signals, cameras, at=None):
    at = time.monotonic() if at is None else at
    with signals.lock:
        signals.health = signals.controllers = signals.world_lock = (at, True)
        signals.tracking = at
    for key, size in [('state',8), ('arm',7), ('gripper',1)]: signals.add(key, at, [0.0]*size)
    for c in cameras.values(): c.frames = (c.frames+[(at, b'jpeg')])[-30:]


class Checks(unittest.TestCase):
    def test_stale_and_faulted_feedback_rejected(self):
        s = Signals(); now = time.monotonic(); feed(s, {}, now)
        self.assertEqual(len(s.sample(now, now)['state'].values), 8)
        with self.assertRaises(ValueError): s.sample(now, now+1)
        s.health = (now, False)
        with self.assertRaises(ValueError): s.sample(now, now)

    def test_capture_review_discard_and_fault(self):
        with tempfile.TemporaryDirectory() as tmp:
            cams = {'main':Camera(), 'right':Camera()}
            r = Recorder(tmp, cams, 'test', sys.executable)
            stop = threading.Event()
            def producer():
                while not stop.wait(1/30): feed(r.signals, cams)
            thread = threading.Thread(target=producer); thread.start()
            try:
                time.sleep(.25)
                r.command('start'); time.sleep(.6)
                self.assertEqual(r.state, 'recording', r.message)
                r.command('stop'); time.sleep(.05)
                self.assertEqual(r.state, 'review', r.message)
                rows = [json.loads(l) for l in (r.path/'frames.jsonl').read_text().splitlines()]
                self.assertGreater(len(rows), 3)
                self.assertEqual([v['index'] for v in rows], list(range(len(rows))))
                r.command('discard'); time.sleep(.05)
                self.assertEqual(r.state, 'idle')
                r.command('start'); time.sleep(.25)
                stop.set(); thread.join(); time.sleep(.25)
                self.assertEqual(r.state, 'fault')
                r.command('save'); time.sleep(.05)
                self.assertEqual(r.state, 'fault')
                self.assertEqual(r.saved, 0)
            finally:
                stop.set(); thread.join(); r.close()

    def test_discard_home_uses_existing_routine_and_cancel_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            cams = {'main':Camera(), 'right':Camera()}
            r = Recorder(tmp, cams, 'test', sys.executable)
            r.stop.set(); r.thread.join()
            process = MagicMock()
            process.poll.return_value = None
            try:
                feed(r.signals, cams, time.monotonic()-.10)
                r.begin()
                pending = r.path
                with patch('openarm_episode.subprocess.Popen', return_value=process) as launch:
                    r.handle('discard_home', 'recording')
                self.assertFalse(pending.exists())
                self.assertEqual(r.state, 'returning')
                command = launch.call_args.args[0]
                self.assertTrue(command[1].endswith('openarm_return_to_zero.py'))
                self.assertEqual(command[2:], ['--execute'])
                r.cancel_home(); r.cancel_home()
                self.assertEqual(process.send_signal.call_count, 1)
                process.poll.return_value = 0
            finally: r.close()

    def test_camera_skew_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            cams = {'main':Camera(), 'right':Camera()}
            r = Recorder(tmp, cams, 'test', sys.executable)
            r.stop.set(); r.thread.join()
            try:
                now = time.monotonic(); feed(r.signals, cams, now)
                cams['right'].frames = [(now-.2,b'jpeg')]
                with self.assertRaises(ValueError): r.snapshot(now)
            finally: r.close()


if __name__ == '__main__': unittest.main()
