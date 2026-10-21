"""Verify a panel child can prompt through /dev/tty, as sudo does."""
import errno
import json
import os
from pathlib import Path
import pty
import select
import shlex
import signal
import sys
import tempfile
import time

with tempfile.TemporaryDirectory() as directory:
    path=Path(directory)
    probe='import os; t=os.open("/dev/tty",os.O_RDWR); assert os.tcgetpgrp(t)==os.getpgrp(); os.write(t,b"TTY_PROMPT\\n"); value=os.read(t,128).decode().strip(); print("TTY_OK" if value=="test-input" else "TTY_BAD",flush=True)'
    (path/'command.json').write_text(json.dumps({'command':shlex.quote(sys.executable)+' -c '+shlex.quote(probe)}))
    pid,fd=pty.fork()
    if pid==0:
        os.execl(sys.executable,sys.executable,str(Path(__file__).parents[1]/'scripts/openarm_panel_job.py'),directory)
    output=b''; sent=False
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            if not select.select([fd],[],[],.1)[0]: continue
            try: chunk=os.read(fd,4096)
            except OSError as exc:
                if exc.errno==errno.EIO: break
                raise
            output+=chunk
            if b'TTY_PROMPT' in output and not sent:
                os.write(fd,b'test-input\n');sent=True
            if b'Press Enter to close.' in output:
                os.write(fd,b'\n');break
        assert b'TTY_OK' in output,output.decode(errors='replace')
        assert json.loads((path/'state.json').read_text())['code']==0
        print('Terminal foreground, /dev/tty prompt and response passed; no sudo or robot commands executed.')
    finally:
        os.close(fd)
        try: os.kill(pid,signal.SIGHUP)
        except ProcessLookupError: pass
        os.waitpid(pid,0)
