"""Explicit USB preflight/ADB install; no ROS imports, factory services or motion."""
import argparse,json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from tg3_pico_dualarm.usb_preflight import install_missing_adb,wait_for_usb

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--packages',type=Path)
    parser.add_argument('--manifest-sha');parser.add_argument('--install-missing',action='store_true')
    parser.add_argument('--wait',type=float,default=60.);args=parser.parse_args()
    if not 0<=args.wait<=60:parser.error('wait must be 0..60 seconds')
    try:
        result={}
        if args.install_missing:
            if not args.packages or not args.manifest_sha:raise ValueError('ADB离线缓存尚未准备好')
            result=install_missing_adb(args.packages,args.manifest_sha)
        identity=wait_for_usb(args.wait,notify=lambda message:print(message,flush=True))
        print(json.dumps(dict(event='robot_usb_preflight',ready=True,operator_uid=os.geteuid(),
            usb=identity,dependencies=result,robot_commands_sent=False)),flush=True);return 0
    except Exception as exc:
        print(json.dumps(dict(event='robot_usb_preflight',ready=False,reason=str(exc),robot_commands_sent=False)),flush=True);return 2

if __name__=='__main__':raise SystemExit(main())
