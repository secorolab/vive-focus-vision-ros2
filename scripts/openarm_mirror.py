#!/usr/bin/env python3
"""Open the established one-eye Vive desktop mirror (separate from dataset FPS)."""
import argparse
import subprocess
import sys


def select_device(output):
    devices=[]
    for line in output.splitlines():
        fields=line.split()
        if len(fields)>=2 and fields[1]=='device': devices.append(fields[0])
    usb=[d for d in devices if ':' not in d]
    choices=usb or devices
    if len(choices)!=1:
        raise ValueError('Connect and authorize the Vive via USB, or connect exactly one wireless ADB device. Use --serial if several devices are connected.')
    return choices[0]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial')
    args=parser.parse_args()
    try:
        serial=args.serial or select_device(subprocess.check_output(['adb','devices'],text=True,timeout=8))
        return subprocess.call(['scrcpy','--serial',serial,'--crop=2448:2448:0:0',
                                '--max-size=960','--max-fps=60','--window-title=Vive - one eye'])
    except (OSError,ValueError,subprocess.SubprocessError) as exc:
        print(f'Cannot open Vive mirror: {exc}',file=sys.stderr); return 1


if __name__=='__main__': raise SystemExit(main())
