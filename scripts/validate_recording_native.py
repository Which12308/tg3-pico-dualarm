"""Explicit Linux ROS schema/CDR check. No camera start, files of samples, or motion."""
import argparse,hashlib,json,sys,time,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--seconds',type=float,default=3.)
    parser.add_argument('--topics',nargs='*');args=parser.parse_args()
    if not 1<=args.seconds<=30:parser.error('seconds must be in 1..30')
    result=dict(passed=False,scope='readonly_ROS_topic_discovery_installed_schema_CDR_decode_only',
        recording_started=False,robot_commands_sent=False,physical_usb_tested=False)
    try:
        if sys.platform!='linux':raise RuntimeError('requires_native_Linux_ROS2_not_executed')
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from tg3_pico_dualarm.recording_exporter import catalogue,message_schema
        found=catalogue();selected={x['topic']:x['type'] for x in found if
            (x['topic'] in args.topics if args.topics else not x['camera'])}
        if not selected:raise RuntimeError('no_selected_live_topics')
        if args.topics and set(args.topics)-selected.keys():raise RuntimeError('requested_topic_missing')
        result['schemas']={name:dict(type=typ,sha256=hashlib.sha256(message_schema(typ)).hexdigest()) for name,typ in selected.items()}
        counts=dict.fromkeys(selected,0);errors=[];context=Context();rclpy.init(args=[],context=context)
        node=Node('tg3_recording_native_check_'+uuid.uuid4().hex[:8],context=context,enable_rosout=False,start_parameter_services=False)
        def callback_for(topic,typ):
            def callback(raw):
                try:deserialize_message(bytes(raw),get_message(typ));counts[topic]+=1
                except Exception as exc:errors.append(topic+':'+str(exc))
            return callback
        try:
            for topic,typ in selected.items():node.create_subscription(get_message(typ),topic,callback_for(topic,typ),qos_profile_sensor_data,raw=True)
            deadline=time.monotonic()+args.seconds
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        finally:node.destroy_node();context.shutdown()
        result.update(messages=counts,errors=errors,passed=not errors and any(counts.values()))
    except Exception as exc:result['error']=str(exc)
    target=ROOT/'artifacts/manual_recording_native_result.json';target.parent.mkdir(exist_ok=True)
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['passed'] else 2

if __name__=='__main__':raise SystemExit(main())
