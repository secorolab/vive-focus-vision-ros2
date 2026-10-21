"""Check graceful stopping of an owned dummy job; never touches ROS/hardware."""
import importlib.util
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import time
spec=importlib.util.spec_from_file_location('job',Path(__file__).parents[1]/'scripts/openarm_panel_job.py')
job=importlib.util.module_from_spec(spec);spec.loader.exec_module(job)
with tempfile.TemporaryDirectory() as directory:
    path=Path(directory)
    code='import signal,time,sys; signal.signal(signal.SIGINT, lambda *a: sys.exit(0)); print("ready",flush=True); time.sleep(30)'
    (path/'command.json').write_text(json.dumps({'command':shlex.quote(sys.executable)+' -c '+shlex.quote(code)}))
    process=subprocess.Popen([sys.executable,str(Path(job.__file__)),directory],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,text=True)
    try:
        assert process.stdout.readline().strip()=='ready'
        deadline=time.monotonic()+3
        while not (path/'state.json').exists() and time.monotonic()<deadline: time.sleep(.02)
        original=json.loads((path/'state.json').read_text())
        (path/'state.json').write_text(json.dumps({**original,'token':'invalid'}))
        assert not job.stop_job(path/'state.json')
        (path/'state.json').write_text(json.dumps(original))
        assert job.stop_job(path/'state.json')
        process.wait(timeout=5)
        state=json.loads((path/'state.json').read_text())
        assert state['state']=='finished' and state['code']==0,state
        assert not job.stop_job(path/'state.json')
    finally:
        if process.poll() is None:
            job.stop_job(path/'state.json');process.wait(timeout=5)
print('Owned job stops gracefully; stale identity and finished jobs are not signalled.')
