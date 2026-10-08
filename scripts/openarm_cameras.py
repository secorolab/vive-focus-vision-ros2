#!/usr/bin/env python3
"""Main + left-wrist RealSense color preview for VR. No ROS, recording or motor control."""
import argparse
import json
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import cv2


def discover(model, index):
    matches = sorted(Path('/dev/v4l/by-id').glob(f'*Camera_{model}_*video-index{index}'))
    if len(matches) > 1:
        raise ValueError(f'Multiple D{model} cameras: select a device explicitly')
    return str(matches[0]) if matches else None


class Camera:
    def __init__(self, name, device, fps=15, model=None, index=0):
        self.name, self.device, self.fps = name, device, fps
        self.model, self.index = model, index
        self.latest = None
        self.error = 'Camera not connected' if not device else 'Opening camera'
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            if not self.device and self.model:
                try: self.device = discover(self.model, self.index)
                except ValueError as exc: self.error = str(exc)
            if not self.device:
                self.stop.wait(2)
                continue
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            try:
                if not cap.isOpened():
                    raise RuntimeError('Cannot open camera; check USB connection or another camera application')
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                while not self.stop.is_set():
                    okay, frame = cap.read()
                    at = time.monotonic()
                    if not okay:
                        raise RuntimeError('Camera disconnected / no image')
                    if frame.shape != (480, 640, 3):
                        raise RuntimeError(f'Expected 640x480 RGB, received {frame.shape}')
                    okay, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
                    if not okay: raise RuntimeError('JPEG encoding failed')
                    self.latest = (at, jpeg.tobytes())
                    self.error = ''
            except Exception as exc:
                self.error = str(exc)
                self.latest = None
            finally:
                cap.release()
            self.stop.wait(2)

    def status(self):
        frame = self.latest
        age = time.monotonic()-frame[0] if frame else None
        return dict(label=self.name, live=age is not None and age < .5 and not self.error,
                    age_s=age, error=self.error, width=640, height=480)


PAGE = b'''<!doctype html><meta name="viewport" content="width=device-width"><title>OpenArm cameras</title>
<style>body{font:18px system-ui;background:#121820;color:#eee;margin:30px}main{display:flex;gap:20px;flex-wrap:wrap}img{width:min(640px,90vw)}p{color:#9cd}</style>
<h1>OpenArm camera preview</h1><p>Live color views only. No recording or robot control.</p><main>
<section><h2>Main camera (D455f)</h2><img id="main"><p id="main-status"></p></section>
<section><h2>Left wrist (D405)</h2><img id="wrist"><p id="wrist-status"></p></section><section><h2>Right wrist</h2><img id="right"><p id="right-status"></p></section></main>
<script>
for(const key of ['main','wrist','right']) {const im=document.getElementById(key);
const next=()=>setTimeout(()=>im.src='/cameras/'+key+'.jpg?t='+Date.now(),125);
im.onload=()=>{im.style.opacity=1;next()};im.onerror=()=>{im.style.opacity=.2;next()};next();}
setInterval(async()=>{try{const s=await(await fetch('/status',{cache:'no-store'})).json();
for(const key of ['main','wrist','right']){const c=s.cameras[key];document.getElementById(key+'-status').textContent=c.live?'LIVE':c.error||'STALE';}}
catch(e){for(const key of ['main','wrist','right'])document.getElementById(key+'-status').textContent='SERVER DISCONNECTED';}},500);
</script>'''


def make_handler(cameras):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route = urlsplit(self.path).path
            if route == '/': data, mime = PAGE, 'text/html; charset=utf-8'
            elif route == '/status':
                data = json.dumps(dict(version=1,cameras={k:c.status() for k,c in cameras.items()})).encode()
                mime = 'application/json'
            elif route in ('/cameras/main.jpg', '/cameras/wrist.jpg', '/cameras/right.jpg'):
                camera = cameras[route.split('/')[-1][:-4]]
                frame = camera.latest
                if not frame or time.monotonic()-frame[0] >= .5 or camera.error:
                    self.send_error(503, 'Camera unavailable'); return
                data, mime = frame[1], 'image/jpeg'
            else: self.send_error(404); return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError): pass
        def log_message(self, *args): pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--main', help='D455f RGB device (stable /dev/v4l/by-id/... recommended)')
    parser.add_argument('--wrist', help='D405 RGB device (stable /dev/v4l/by-id/... recommended)')
    parser.add_argument('--right', help='Optional right-wrist RGB device; use a stable by-id path')
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--bind', default='0.0.0.0', help='Listen address; LAN access is needed by headset')
    parser.add_argument('--fps', type=int, choices=[15,30], default=15)
    args=parser.parse_args()
    try:
        main_device=args.main or discover('455f',0)
        wrist_device=args.wrist or discover('405',4)
    except ValueError as exc: parser.error(str(exc))
    devices = [str(Path(p).resolve()) for p in (main_device, wrist_device, args.right) if p]
    if len(devices) != len(set(devices)): parser.error('Each camera must use a different device')
    if main_device and wrist_device and Path(main_device).resolve()==Path(wrist_device).resolve():
        parser.error('Main and wrist must be different devices')
    cv2.setNumThreads(1)
    cameras={'main':Camera('Main camera (D455f)',main_device,args.fps,'455f',0),
             'wrist':Camera('Left wrist (D405)',wrist_device,args.fps,'405',4),
             'right':Camera('Right wrist',args.right,args.fps)}
    server = ThreadingHTTPServer((args.bind,args.port),make_handler(cameras))
    server.daemon_threads = True
    print(f'Main: {main_device}\nLeft wrist: {wrist_device}\nPreview: http://localhost:{args.port}\nKeep this running while using VR. Ctrl+C stops cameras only.',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        for camera in cameras.values(): camera.stop.set()
        server.server_close()
        for camera in cameras.values(): camera.thread.join(timeout=2)

if __name__=='__main__': main()
