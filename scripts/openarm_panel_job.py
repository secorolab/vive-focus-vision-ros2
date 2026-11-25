#!/usr/bin/env python3
"""Run one panel-owned command in its own process group, with inspectable state."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def start_token(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except (OSError, IndexError):
        return None


def stop_job(state_path):
    state=json.loads(Path(state_path).read_text())
    if state.get('state')!='running': return False
    pid=state['pid']
    if start_token(pid)!=state['token'] or os.getpgid(pid)!=pid: return False
    os.killpg(pid,signal.SIGINT)
    return True


def run(directory):
    directory=Path(directory)
    command=json.loads((directory/'command.json').read_text())['command']
    # A separate process group supports Stop, but a new session would lose the
    # controlling terminal and prevent sudo from opening /dev/tty.
    tty_fd = sys.stdin.fileno() if sys.stdin.isatty() else None
    foreground = os.tcgetpgrp(tty_fd) if tty_fd is not None else None
    def set_foreground(group):
        if tty_fd is None: return
        previous = signal.signal(signal.SIGTTOU, signal.SIG_IGN)
        try: os.tcsetpgrp(tty_fd, group)
        finally: signal.signal(signal.SIGTTOU, previous)
    process=subprocess.Popen(['bash','-c',command],process_group=0)
    set_foreground(process.pid)
    # A very early terminal read can stop the child before foreground transfer.
    if process.poll() is None: os.killpg(process.pid, signal.SIGCONT)
    def save(state):
        temporary=directory/'state.tmp'
        temporary.write_text(json.dumps(state)); temporary.replace(directory/'state.json')
    save(dict(state='running',pid=process.pid,token=start_token(process.pid)))
    try:
        while True:
            try:
                code=process.wait(); break
            except KeyboardInterrupt:
                if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
    finally:
        if foreground is not None: set_foreground(foreground)
        if process.poll() is not None: save(dict(state='finished',code=process.returncode))
    print(f'Command finished (exit {code}). You can close this terminal.',flush=True)
    try: input('Press Enter to close. ')
    except (EOFError,KeyboardInterrupt): pass


if __name__=='__main__': run(sys.argv[1])
