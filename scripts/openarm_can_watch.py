#!/usr/bin/env python3
"""Listen to OpenArm CAN traffic without transmitting or enabling motors."""
import argparse
import collections
import json
import select
import socket
import time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=90)
    parser.add_argument('--output', type=Path, default=Path('/tmp/openarm_can_watch.jsonl'))
    args=parser.parse_args()
    if not 0 < args.seconds <= 3600:
        parser.error('--seconds must be between 0 and 3600')
    sockets={}
    counts=collections.Counter(); last={}; gaps=collections.defaultdict(float)
    previous=collections.Counter(); reported=set()
    try:
        for interface in ('can0','can1'):
            bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
            sockets[bus]=interface
            bus.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FD_FRAMES,1)
            bus.bind((interface,))
        with args.output.open('w') as out:
            print(f'Listening only on can0/right and can1/left for {args.seconds:g}s. Log: {args.output}',flush=True)
            start=time.monotonic(); report=start+1; end=start+args.seconds
            while time.monotonic()<end:
                for bus in select.select(list(sockets),[],[],0.01)[0]:
                    frame=bus.recv(72)
                    ident=int.from_bytes(frame[:4],'little')
                    if ident not in list(range(1,9))+list(range(17,25)):
                        continue
                    key=(sockets[bus],ident)
                    now=time.monotonic()
                    counts[key]+=1
                    if key in last: gaps[key]=max(gaps[key],now-last[key])
                    last[key]=now
                now=time.monotonic()
                for interface in sockets.values():
                    for motor in range(1,9):
                        tx=(interface,motor); rx=(interface,motor+16)
                        missing=(tx in last and now-last[tx]<0.05 and
                                 now-last.get(rx,start)>0.1)
                        if missing and tx not in reported:
                            event={'event':'commands_seen_without_recent_reply','interface':interface,
                                   'motor':motor,'elapsed_s':now-start,
                                   'reply_age_s':now-last[rx] if rx in last else None}
                            out.write(json.dumps(event)+'\n'); out.flush()
                            print(json.dumps(event),flush=True); reported.add(tx)
                        elif not missing: reported.discard(tx)
                if now>=report:
                    rows=[]
                    for interface in sockets.values():
                        for motor in range(1,9):
                            tx=(interface,motor); rx=(interface,motor+16)
                            rows.append({'interface':interface,'motor':motor,
                                         'commands':counts[tx]-previous[tx],
                                         'replies':counts[rx]-previous[rx],
                                         'reply_age_ms':(now-last[rx])*1000 if rx in last else None,
                                         'max_reply_gap_ms':gaps[rx]*1000})
                    out.write(json.dumps({'elapsed_s':now-start,'motors':rows})+'\n'); out.flush()
                    previous=counts.copy(); report=now+1
            print(f'Capture complete: {args.output}',flush=True)
    except OSError as exc:
        parser.exit(1, f'CAN capture unavailable: {exc}. Configure and bring up both CAN interfaces before capturing.\n')
    except KeyboardInterrupt:
        print(f'Capture stopped: {args.output}')
    finally:
        for bus in sockets: bus.close()


if __name__=='__main__':
    main()
