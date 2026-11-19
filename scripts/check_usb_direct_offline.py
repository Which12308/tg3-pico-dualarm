"""Real Java sender -> Python USB topology over localhost. No ADB, SSH, ROS or motion."""
import hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer

def main():
    result=dict(passed=False,scope='actual_Java_sender_and_Python_localhost_gateway_synthetic_poses',
        headset_tested=False,usb_hardware_tested=False,robot_commands_sent=False,recording_started=False)
    server=None;folder=ROOT/'artifacts/usb_direct_jvm_python';folder.mkdir(parents=True,exist_ok=True)
    try:
        java=shutil.which('java');javac=shutil.which('javac')
        jar=ROOT.parent/'native_tools/java_test_dependencies/json-20240303.jar'
        expected='3cf6cd6892e32e2b4c1c39e0f52f5248a2f5b37646fdfbb79a66b46b618414ed'
        if not java or not javac or not jar.exists():raise RuntimeError('JDK_or_pinned_JSON_jar_unavailable')
        if hashlib.sha256(jar.read_bytes()).hexdigest()!=expected:raise RuntimeError('JSON_jar_hash_mismatch')
        sources=[ROOT/'headset_native/java/edu/scu/tg3/wireless/WirelessLink.java',ROOT/'headset_native/tests/PythonGatewayJvmTest.java']
        subprocess.run([javac,'--release','8','-encoding','UTF-8','-classpath',str(jar),'-d',str(folder),*map(str,sources)],check=True,capture_output=True)
        ages=[];sequences=[];active=[]
        def observed(frame,_):
            if frame.focused:
                ages.append(frame.source_age_s);sequences.append(frame.sequence);active.append(frame.right.grip)
        peer=InputGateway(on_frame=observed);control=InputGateway(role='relay',key='offline-test-key-'+'x'*32)
        control.tracking=peer.tracking;control.accept_frames=False;control.headset_gateway=peer
        server=WirelessServer(peer,'127.0.0.1',8877).start();pin=peer.open_pairing()
        proc=subprocess.run([java,'-classpath',str(folder)+os.pathsep+str(jar),'edu.scu.tg3.wireless.PythonGatewayJvmTest'],
            input=pin+'\n',capture_output=True,text=True,encoding='utf-8',timeout=20)
        if proc.returncode:raise RuntimeError(proc.stderr[-2000:])
        if len(ages)<20 or not any(v>0 for v in active):raise RuntimeError('insufficient_actual_Java_frames')
        if not all(b>a for a,b in zip(sequences,sequences[1:])):raise RuntimeError('sequence_order_failed')
        if not all(0<=a<=.12 for a in ages):raise RuntimeError('cumulative_budget_failed')
        # Java close invalidates input; software cannot treat subsequent silence as live tracking.
        if control.tracking.sample().focused:raise RuntimeError('sender_close_did_not_invalidate')
        result.update(passed=True,actual_accepted_active_frames=len(ages),maximum_source_and_transport_age_ms=max(ages)*1000,
            strictly_increasing_sequences=True,shared_direct_tracking=True,close_invalidates=True,
            java_source_sha256=hashlib.sha256(sources[0].read_bytes()).hexdigest(),gateway_source_sha256=hashlib.sha256((ROOT/'src/tg3_pico_dualarm/wireless.py').read_bytes()).hexdigest(),
            automatic_recording=False)
    except Exception as exc:result['error']=str(exc)
    finally:
        if server:server.close()
    (ROOT/'artifacts/usb_direct_offline_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['passed'] else 2

if __name__=='__main__':raise SystemExit(main())
