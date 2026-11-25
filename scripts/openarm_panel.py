#!/usr/bin/env python3
"""Local desktop launcher. Motor operations stay visible in dedicated terminals."""
import json
import os
import shlex
import subprocess
import tempfile
import threading
import queue
import time
import urllib.request
from openarm_panel_job import stop_job, start_token
from openarm_export_lerobot import export_counts
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

REPO = Path(__file__).resolve().parents[1]
SETUP = ('source /opt/ros/jazzy/setup.bash && '
         'source "$HOME/openarm_ws/install/setup.bash" && '
         'source "$HOME/vive_vr_ws/install/setup.bash" && ')
DOMAIN = 'ROS_DOMAIN_ID=84 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST '

def robot_running():
    for p in Path('/proc').glob('[0-9]*'):
        try:
            name = Path(p.joinpath('cmdline').read_bytes().split(b'\0')[0].decode()).name
        except (OSError, UnicodeError):
            continue
        if name == 'ros2_control_node': return True
    return False


def main():
    root = tk.Tk(); root.title('OpenArm controls'); root.geometry('820x800')
    canvas = tk.Canvas(root, highlightthickness=0)
    scrollbar = ttk.Scrollbar(root, orient='vertical', command=canvas.yview)
    scrollbar.pack(side='right', fill='y')
    canvas.pack(side='left', fill='both', expand=True)
    canvas.configure(yscrollcommand=scrollbar.set)
    frame = ttk.Frame(canvas, padding=20)
    content = canvas.create_window((0, 0), window=frame, anchor='nw')
    frame.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
    canvas.bind('<Configure>', lambda event: canvas.itemconfigure(content, width=event.width))
    def scroll(event):
        if canvas.bbox('all')[3] > canvas.winfo_height():
            direction = -1 if getattr(event, 'num', None) == 4 or event.delta > 0 else 1
            canvas.yview_scroll(direction*3, 'units')
        return 'break'
    root.bind_all('<Button-4>', scroll)
    root.bind_all('<Button-5>', scroll)
    root.bind_all('<MouseWheel>', scroll)
    ttk.Label(frame, text='OpenArm controls', font=('',20,'bold')).pack(anchor='w')
    ttk.Label(frame, text='Home without a headset. Start VR separately when ready.').pack(anchor='w',pady=(5,12))
    ip = tk.StringVar(value='10.23.1.157')
    try:
        interfaces=json.loads(subprocess.check_output(['ip','-j','-4','addr','show'],text=True))
        for interface in interfaces:
            if interface['ifname'].startswith('wl'):
                for address in interface.get('addr_info',[]):
                    if address.get('scope')=='global': ip.set(address['local'])
    except (OSError, subprocess.SubprocessError, ValueError): pass
    row=ttk.Frame(frame); row.pack(fill='x')
    ttk.Label(row,text='Laptop Wi-Fi IP (VR only): ').pack(side='left')
    ttk.Entry(row,textvariable=ip,width=20).pack(side='left')
    close=tk.BooleanVar(value=False)
    ttk.Checkbutton(frame,text='Also close both grippers during homing (empty grippers only)',variable=close).pack(anchor='w',pady=10)
    mirror=tk.BooleanVar(value=True)
    ttk.Checkbutton(frame,text='Open Vive one-eye scrcpy view with cameras (60 FPS mirror)',variable=mirror).pack(anchor='w')
    status=tk.StringVar(); ttk.Label(frame,textvariable=status).pack(anchor='w',pady=5)
    terminals={}
    jobs={}
    running=ttk.Treeview(frame,columns=('state',),show='tree headings',height=4)
    running.heading('#0',text='Command'); running.heading('state',text='Status')
    def terminal(title, command, exclusive=False):
        if title in jobs and job_state(title)=='running':
            messagebox.showinfo('Already open',f'The {title} terminal is already open. Use that terminal.'); return
        if exclusive and robot_running():
            messagebox.showerror('Robot already running','Support the arms, then Ctrl+C in the existing robot/home terminal first.'); return
        directory=Path(tempfile.mkdtemp(prefix='openarm-panel-'))
        (directory/'command.json').write_text(json.dumps({'command':command, 'title':title}))
        jobs[title]=directory
        if running.exists(title): running.delete(title)
        running.insert('', 'end', iid=title, text=title, values=('Starting',))
        try:
            terminals[title]=subprocess.Popen(['gnome-terminal','--wait','--title='+title,'--',
                '/usr/bin/python3',str(REPO/'scripts/openarm_panel_job.py'),str(directory)])
        except OSError as e: messagebox.showerror('Could not open terminal',str(e))
    def job_state(title):
        try: return json.loads((jobs[title]/'state.json').read_text())['state']
        except (OSError, ValueError, KeyError):
            return 'running' if title in terminals and terminals[title].poll() is None else 'not started'
    def stop_selected():
        selected=running.selection()
        if not selected: return
        try:
            stopped=stop_job(jobs[selected[0]]/'state.json')
            if not stopped: messagebox.showinfo('Not running','This command has finished or is no longer owned by this panel.')
        except (OSError, ValueError, KeyError) as exc: messagebox.showinfo('Could not stop',str(exc))
    def py(name): return '/usr/bin/python3 '+shlex.quote(str(REPO/'scripts'/name))
    def home():
        terminal('OpenArm home', 'sudo prlimit --pid $$ --rtprio=50:50 && '+SETUP+py('openarm_home.py')+' --execute'+(' --close-grippers' if close.get() else ''),True)
    def vr():
        import ipaddress
        try: host=str(ipaddress.IPv4Address(ip.get().strip()))
        except ValueError: messagebox.showerror('Invalid IP','Enter the laptop Wi-Fi IPv4 address.'); return
        terminal('OpenArm VR','sudo prlimit --pid $$ --rtprio=50:50 && '+SETUP+'ros2 launch vive_vr_ros2 openarm_real.launch.py host_ip:='+host,True)
    def open_mirror():
        if 'Vive mirror' not in jobs or job_state('Vive mirror')!='running':
            terminal('Vive mirror',py('openarm_mirror.py'))
    def cameras():
        if 'Cameras and recorder' not in jobs or job_state('Cameras and recorder')!='running':
            terminal('Cameras and recorder',SETUP+py('openarm_cameras.py')+' --fps 60 --record-fps 30 --record-root "$HOME/robot_ai/episodes/right_cube_30hz" --task "Put the cube in the basket"')
        if mirror.get(): open_mirror()
    def camera_preview():
        import webbrowser
        webbrowser.open('http://127.0.0.1:8081')
    can=' && '.join(['sudo ip link set '+bus+' '+operation for operation in (
        'down','type can bitrate 1000000 dbitrate 5000000 fd on','txqueuelen 32','up') for bus in ('can0','can1')])
    buttons=[
        ('1. Set up CAN',lambda:terminal('CAN setup',can,True)),
        ('2. Return to zero + keep holding',home),
        ('Read joint positions (no movement)',lambda:terminal('Joint status',SETUP+py('openarm_return_to_zero.py')+' --status')),
        ('Explain startup problem (no movement)',lambda:terminal('Startup status',SETUP+py('openarm_retry_startup.py'))),
        ('3. Start VR robot (after stopping homing)',vr),
        ('4. Start cameras + 30 Hz recorder + Vive view',cameras),
        ('Open Vive mirror only (one eye)',open_mirror),
        ('Open RealSense camera preview in browser',camera_preview),
        ('5. Enable VR arms + grippers (world locked, buttons released)',lambda:terminal('Enable VR',SETUP+DOMAIN+"ros2 service call /openarm_hardware_preview/enable std_srvs/srv/SetBool '{data: true}'")),
        ('Disable VR (keep holding)',lambda:terminal('Disable VR',SETUP+DOMAIN+"ros2 service call /openarm_hardware_preview/enable std_srvs/srv/SetBool '{data: false}'")),
    ]
    def calibrate(side):
        command = SETUP+DOMAIN+py('openarm_align.py')+' --domain-id 84 --mode real --arm '+side+' --output "$HOME/vive_vr_ws/models/openarm_v1" --delay 10'
        terminal(side.capitalize()+' wrist calibration',command)
    buttons += [('Align RIGHT end-effector pose (10-second countdown)',lambda:calibrate('right')),
                ('Align LEFT end-effector pose (10-second countdown)',lambda:calibrate('left'))]
    for text, action in buttons:
        ttk.Button(frame,text=text,command=action).pack(fill='x',pady=3)
    ttk.Label(frame,wraplength=770,text='Pose alignment: lock world, click the arm button, then release side grips. You have 10 seconds to hold the controller in the desired tool orientation. Keep it still during capture. Alignment applies immediately; release side grips, then click Enable VR. No launch restart needed.\nStop selected sends Ctrl+C. SUPPORT ARMS FIRST: stopping home/VR can release torque.\nCAN setup refuses an active robot; stop its robot terminal first. Closing this panel leaves jobs running.').pack(anchor='w',pady=8)
    running.pack(fill='x')
    ttk.Button(frame,text='Stop selected running command (Ctrl+C)',command=stop_selected).pack(fill='x',pady=4)
    record_label=tk.StringVar(value='Recorder: checking…')
    ttk.Label(frame,textvariable=record_label,wraplength=770).pack(anchor='w',pady=6)
    commands=queue.Queue(); updates=queue.Queue(); last_update=0; latest={}; pending=False
    def worker():
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while True:
            try:
                action=commands.get_nowait()
            except queue.Empty: action=None
            try:
                if action:
                    request=urllib.request.Request('http://127.0.0.1:8081/recording/command',
                        data=json.dumps({'action':action}).encode(),headers={'Content-Type':'application/json'},method='POST')
                    with opener.open(request,timeout=2) as response: response.read()
                with opener.open('http://127.0.0.1:8081/recording/status',timeout=1) as response:
                    updates.put((time.monotonic(),json.load(response),None))
            except Exception as exc: updates.put((time.monotonic(),{},str(exc)))
            time.sleep(.5)
    controls=ttk.Frame(frame); controls.pack(fill='x')
    record_buttons={}
    def record(action):
        nonlocal pending
        if pending or time.monotonic()-last_update>2: return
        pending=True
        commands.put(action)
        for button in record_buttons.values(): button.configure(state='disabled')
    for action,label in [('start','Start recording'),('stop','Stop recording'),('save','Save episode'),('discard','Discard unsaved')]:
        button=ttk.Button(controls,text=label,command=lambda a=action:record(a),state='disabled')
        button.pack(side='left',expand=True,fill='x'); record_buttons[action]=button
    threading.Thread(target=worker,daemon=True).start()
    ttk.Button(frame, text='Export saved episodes to LeRobot (after recording session)',
        command=lambda: terminal('Export LeRobot dataset',
            '"$HOME/robot_ai/lerobot_env/bin/python" ' +
            shlex.quote(str(REPO/'scripts/openarm_export_lerobot.py')) +
            ' "$HOME/robot_ai/episodes/right_cube_30hz"')).pack(fill='x', pady=4)
    ttk.Button(frame, text='Export only newest saved episode (GPU)',
        command=lambda: terminal('Export LeRobot dataset',
            '"$HOME/robot_ai/lerobot_env/bin/python" ' +
            shlex.quote(str(REPO/'scripts/openarm_export_lerobot.py')) +
            ' "$HOME/robot_ai/episodes/right_cube_30hz" --latest')).pack(fill='x', pady=4)
    export_label = tk.StringVar()
    ttk.Label(frame, textvariable=export_label, wraplength=770).pack(anchor='w', pady=4)
    def refresh():
        counts = export_counts(Path.home()/'robot_ai/episodes/right_cube_30hz')
        export_label.set(f"Waiting to export: {counts['waiting']} | Exported: {counts['exported']} | Total saved: {counts['total']}")
        status.set('Robot controller running — use its terminal to stop.' if robot_running() else 'No robot controller detected.')
        nonlocal last_update, latest, pending
        for title in jobs:
            running.item(title,values=(job_state(title),))
        while not updates.empty():
            last_update,latest,error=updates.get_nowait(); pending=False
            if error: record_label.set('Recorder disconnected — recording state unknown. Start/check cameras terminal.')
            else:
                state=latest.get('state','unknown')
                record_label.set(f"Recorder: {state.upper()} | {latest.get('seconds',0)} s | {latest.get('frames',0)} frames | Total saved: {latest.get('saved',0)}\n{latest.get('message','')}")
        state=latest.get('state')
        allowed={'start':state in ('idle','home_done') and latest.get('ready',False),
                 'stop':state=='recording', 'save':state=='review',
                 'discard':state in ('review','fault')}
        for action,button in record_buttons.items():
            button.configure(state='normal' if allowed[action] and not pending and time.monotonic()-last_update<=2 else 'disabled')
        root.after(300,refresh)
    for directory in Path(tempfile.gettempdir()).glob('openarm-panel-*'):
        try:
            if directory.stat().st_uid != os.getuid(): continue
            state = json.loads((directory/'state.json').read_text())
            if state.get('state') != 'running' or start_token(state['pid']) != state['token']: continue
            job = json.loads((directory/'command.json').read_text())
            title = job.get('title')
            if not title:
                for pattern, name in [('openarm_real.launch.py','OpenArm VR'),
                                      ('openarm_cameras.py','Cameras and recorder'),
                                      ('openarm_mirror.py','Vive mirror')]:
                    if pattern in job['command']: title = name; break
            if title and title not in jobs:
                jobs[title] = directory
                running.insert('', 'end', iid=title, text=title, values=('running',))
        except (OSError, ValueError, KeyError):
            continue
    refresh(); root.mainloop()


if __name__=='__main__': main()
