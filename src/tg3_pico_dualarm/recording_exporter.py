"""Explicit, read-only ROS/CDR exporter. No data files and no actuator API."""
import argparse,json,os,queue,re,sys,threading,time,uuid
from pathlib import Path
from .recording_stream import ByteQueue,RateLimiter,encode_packet

def message_schema(typename):
    from ament_index_python.packages import get_package_share_directory
    from rosidl_runtime_py.utilities import get_message
    visited={}
    def visit(name):
        if name in visited:return
        pkg,_,typ=name.split('/')
        text=(Path(get_package_share_directory(pkg))/'msg'/(typ+'.msg')).read_text()
        visited[name]=text
        for field in get_message(name).get_fields_and_field_types().values():
            for p,t in re.findall(r'([A-Za-z]\w*)/(?:msg/)?([A-Za-z]\w*)',field):visit(p+'/msg/'+t)
    visit(typename)
    data=visited.pop(typename)
    for name,text in visited.items():data+='\n'+'='*80+'\nMSG: '+name.replace('/msg/','/')+'\n'+text
    return data.encode()

def catalogue():
    from .rosbag_record import discover_topics,select_topics,CAMERA_TYPES
    return [dict(topic=n,type=t,camera=t in CAMERA_TYPES) for n,t in select_topics(discover_topics(2.)).items()]

def run(config,stream,stdin):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from .wireless import GatewayClient,PROTOCOL
    topics=config['topics'];rate=config['rate'];token=config['token']
    available={v['topic']:v['type'] for v in catalogue()}
    if not isinstance(topics,dict) or any(available.get(n)!=t for n,t in topics.items()):
        raise ValueError('selected_recording_topic_missing_or_changed')
    schemas={n:message_schema(t) for n,t in topics.items()}
    limiter=RateLimiter(rate)
    def send(header,payload=b''):
        packet=encode_packet(header,payload)
        for start in range(0,len(packet),65536):
            part=packet[start:start+65536];limiter.wait(len(part));stream.write(part);stream.flush()
    # Bounded network measurement before any subscriptions or data recording.
    block=bytes(65536)
    for _ in range(32):send(dict(kind='probe'),block)
    send(dict(kind='probe_end',bytes=32*65536))
    if stdin.readline().strip()!='proceed':raise RuntimeError('recording_preflight_cancelled')
    stop=threading.Event();buffer=ByteQueue();errors=[]
    def fail(exc):errors.append(str(exc)[:1000]);stop.set()
    def parent():
        stdin.read();stop.set()
    threading.Thread(target=parent,daemon=True).start()
    def samples():
        client=GatewayClient('127.0.0.1',config.get('port',8878),token,timeout=.5)
        try:
            while not stop.is_set():
                sample=client.request('GET','/v1/recording/sample')
                buffer.put(dict(kind='sample',timestamp_ns=time.time_ns()),json.dumps(sample,allow_nan=False).encode())
                stop.wait(.05)
        except Exception as exc:fail(exc)
        finally:
            try:
                final=client.request('POST','/v1/recording/end',dict(protocol=PROTOCOL))
                if final.get('error'):raise RuntimeError(final['error'])
                if final.get('final_sample') is not None:buffer.put(dict(kind='sample',timestamp_ns=time.time_ns()),json.dumps(final['final_sample']).encode())
            except Exception as exc:fail(exc)
            finally:client.close()
    context=Context();rclpy.init(args=[],context=context)
    node=Node('tg3_manual_export_'+uuid.uuid4().hex[:8],context=context,enable_rosout=False,
              start_parameter_services=False,use_global_arguments=False)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    threads=[]
    try:
        for topic,typ in topics.items():
            send(dict(kind='schema',topic=topic,type=typ),schemas[topic])
            def callback_for(name):
                def callback(raw,info):
                    try:buffer.put(dict(kind='ros',topic=name,timestamp_ns=time.time_ns(),
                        publish_time_ns=max(0,int(getattr(info,'source_timestamp',0)))),bytes(raw))
                    except Exception as exc:fail(exc)
                return callback
            node.create_subscription(get_message(typ),topic,callback_for(topic),qos_profile_sensor_data,raw=True)
        def spin():
            try:
                while not stop.is_set():executor.spin_once(timeout_sec=.05)
            except Exception as exc:fail(exc)
        threads=[threading.Thread(target=spin,daemon=True),threading.Thread(target=samples,daemon=True)]
        for thread in threads:thread.start()
        send(dict(kind='ready',topics=topics))
        while not stop.is_set():
            try:header,payload=buffer.get(.1)
            except queue.Empty:continue
            send(header,payload)
        # A graceful stop closes a complete prefix; no pretending the tail was saved.
        for thread in threads:thread.join(1.)
        if any(thread.is_alive() for thread in threads):raise RuntimeError('recording_producer_stop_timeout')
        if errors:raise RuntimeError(';'.join(errors))
        while buffer.items:
            header,payload=buffer.get();send(header,payload)
        send(dict(kind='end'))
    finally:
        stop.set()
        for thread in threads:thread.join(1.)
        executor.shutdown(timeout_sec=2.);node.destroy_node()
        if context.ok():context.shutdown()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--list',action='store_true');args=parser.parse_args()
    if args.list:print(json.dumps(catalogue()));return
    config=json.loads(sys.stdin.readline(65536))
    try:run(config,sys.stdout.buffer,sys.stdin)
    except Exception as exc:
        try:sys.stdout.buffer.write(encode_packet(dict(kind='error',reason=str(exc)[:1000])));sys.stdout.buffer.flush()
        except OSError:pass
        raise SystemExit(2)
    finally:
        from .wireless import GatewayClient,PROTOCOL
        client=GatewayClient('127.0.0.1',config.get('port',8878),config['token'],timeout=.3)
        try:client.request('POST','/v1/recording/end',dict(protocol=PROTOCOL))
        except Exception:pass
        finally:client.close()

if __name__=='__main__':main()
