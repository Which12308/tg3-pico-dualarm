"""Actual MCAP/disk/loopback checks; never connect to robot or physical USB."""
import importlib.util,io,json,os,struct,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
from tg3_pico_dualarm.diagnostics import DiagnosticsSink
from tg3_pico_dualarm.bounded_output import RingLog
from tg3_pico_dualarm.recording_stream import ByteQueue,RateLimiter,encode_packet,read_packet,MAX_PAYLOAD
from tg3_pico_dualarm.recording_writer import DatasetWriter,benchmark,check_destination
from tg3_pico_dualarm.manual_recording import ManualRecording,exporter_command,worker
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient,PROTOCOL
HAS_MCAP=importlib.util.find_spec('mcap') is not None

class RecordingTests(unittest.TestCase):
    def test_idle_constructs_no_record_process_and_diagnostics_drop_samples(self):
        with tempfile.TemporaryDirectory() as temp:
            manager=ManualRecording();self.assertIsNone(manager.process);manager.stop();self.assertEqual(manager.poll(),{'state':'off'})
            sink=DiagnosticsSink(Path(temp)/'diagnostics',budget=16384)
            for _ in range(100):sink.append({'tracking':'SECRET_POSE','image':'frame'})
            for i in range(100):sink.event('connection_changed',i,{'trajectory':'SECRET_POSE','reason':'x'*500})
            sink.close();files=list(Path(temp).rglob('*'))
            self.assertLessEqual(sum(p.stat().st_size for p in files if p.is_file()),16384)
            self.assertFalse(any(p.suffix in ('.jsonl','.mcap') for p in files))
            self.assertTrue(all('SECRET_POSE' not in p.read_text() for p in files if p.is_file()))
            self.assertIsNone(manager.process)

    def test_binary_log_rotation_bounds_long_lines_and_sessions(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'factory.log'
            for _ in range(4):RingLog(path,budget=16384).write(b'x'*90000)
            self.assertLessEqual(sum(p.stat().st_size for p in path.parent.iterdir()),16384)

    def test_actual_log_consumer_discards_routine_joint_output_and_survives_eof(self):
        import subprocess,sys
        from tg3_pico_dualarm import bounded_output
        with tempfile.TemporaryDirectory() as temp:
            target=Path(temp)/'factory.log'
            output=subprocess.run([sys.executable,bounded_output.__file__,str(target)],input=
                b'positions SECRET_POSE\n[INFO] periodic target SECRET_POSE\n[ERROR] connection lost\n',
                capture_output=True,timeout=5)
            self.assertEqual(output.returncode,0)
            self.assertIn('connection lost',target.read_text());self.assertNotIn('SECRET_POSE',target.read_text())

    def test_stream_binary_roundtrip_and_rejects_corruption(self):
        payload=b'\0\xff\n'*20;head={'kind':'ros','topic':'/a'}
        self.assertEqual(read_packet(io.BytesIO(encode_packet(head,payload))),(head,payload))
        with self.assertRaises(EOFError):read_packet(io.BytesIO(encode_packet(head,payload)[:-1]))
        with self.assertRaises(ValueError):read_packet(io.BytesIO(struct.pack('!II',1,MAX_PAYLOAD+1)))
        with self.assertRaises(ValueError):read_packet(io.BytesIO(struct.pack('!II',2,0)+b'[]'))

    def test_backpressure_and_age_fail_instead_of_unbounded_buffer(self):
        clock=[0.];buffer=ByteQueue(max_bytes=300,max_age=1.,clock=lambda:clock[0])
        buffer.put({'kind':'sample'},bytes(100))
        with self.assertRaisesRegex(RuntimeError,'backlog'):buffer.put({'kind':'sample'},bytes(300))
        clock[0]=1.01
        with self.assertRaisesRegex(RuntimeError,'old'):buffer.get()
        buffer.close()
        with self.assertRaisesRegex(RuntimeError,'closed'):buffer.put({},b'')

    def test_rate_is_bounded_without_using_control_clock(self):
        clock=[0.]
        def sleep(delta):clock[0]+=delta
        limiter=RateLimiter(65536,clock=lambda:clock[0],sleep=sleep)
        for _ in range(5):limiter.wait(65536)
        self.assertEqual(clock[0],4.)
        with self.assertRaises(ValueError):RateLimiter(4*1024*1024+1)

    @unittest.skipUnless(HAS_MCAP,'PC writer requires desktop/requirements.txt; robot exporter does not require MCAP')
    def test_explicit_write_rotates_real_mcap_and_jsonl_and_reads_cdr(self):
        from mcap.reader import make_reader
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);self.assertEqual(list(root.iterdir()),[])
            self.assertGreater(benchmark(root,size=1024*1024),0)
            self.assertEqual(list(root.iterdir()),[])
            writer=DatasetWriter(root,{'topics':{'/number':'std_msgs/msg/Int32'}},segment_bytes=10,reserve=0)
            writer.write({'kind':'schema','topic':'/number','type':'std_msgs/msg/Int32'},b'int32 data\n')
            raw=b'\x00\x01\x00\x00'+struct.pack('<i',42)
            writer.write({'kind':'ros','topic':'/number','timestamp_ns':123,'publish_time_ns':120},raw)
            writer.write({'kind':'sample'},json.dumps({'tracking':{'sequence':7}}).encode())
            writer.write({'kind':'ros','topic':'/number','timestamp_ns':124},raw)
            writer.close();manifest=json.loads((writer.directory/'session.json').read_text())
            self.assertEqual(manifest['state'],'complete');self.assertFalse(manifest['incomplete'])
            messages=[]
            for file in writer.directory.glob('*.mcap'):
                with file.open('rb') as source:messages.extend(make_reader(source).iter_messages())
            self.assertEqual(len(messages),2)
            self.assertTrue(all(s.encoding=='ros2msg' and c.message_encoding=='cdr' and m.data==raw for s,c,m in messages))
            self.assertEqual([m.log_time for _,_,m in messages],[123,124])
            lines=[json.loads(line) for p in writer.directory.glob('*.jsonl') for line in p.read_text().splitlines()]
            self.assertEqual(lines[0]['source']['tracking']['sequence'],7)
            self.assertGreater(manifest['segments'],1)

    @unittest.skipUnless(HAS_MCAP,'PC writer requires desktop/requirements.txt; robot exporter does not require MCAP')
    def test_destination_removed_never_recreates_it_or_falls_back(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'usb';root.mkdir();writer=DatasetWriter(root,{},reserve=0)
            # Represents disappearing mount: move entire subtree out of the selected path.
            moved=root.with_name('removed');root.rename(moved) if os.name!='nt' else None
            if os.name=='nt':
                # Windows open files prohibit renaming this fixture: exercise the same
                # destination check after finalizing handles, not a pretend physical unplug.
                writer._finish_segment();root.rename(moved)
            with self.assertRaises(OSError):writer.write({'kind':'sample'},b'{}')
            with self.assertRaises(OSError):writer.close('destination_missing')
            self.assertFalse(root.exists());self.assertEqual(set(Path(temp).iterdir()),{moved})

    def test_space_and_volume_change_are_recording_failures(self):
        with tempfile.TemporaryDirectory() as temp:
            with patch('tg3_pico_dualarm.recording_writer.shutil.disk_usage',return_value=type('Usage',(),{'free':1})()):
                with self.assertRaisesRegex(OSError,'space_low'):check_destination(temp)
            with self.assertRaisesRegex(OSError,'volume_changed'):check_destination(temp,device=-1,reserve=0)

    def test_worker_failed_destination_only_reports_recording_error(self):
        import queue,threading
        status=queue.Queue()
        with tempfile.TemporaryDirectory() as temp:
            worker({'destination':str(Path(temp)/'missing')},threading.Event(),status)
            messages=[]
            while not status.empty():messages.append(status.get())
            self.assertEqual(messages[-1]['state'],'error');self.assertTrue(messages[-1]['incomplete'])
            self.assertEqual(list(Path(temp).iterdir()),[])

    def test_exporter_uses_separate_readonly_command_and_no_secret_argv(self):
        command=exporter_command('/home/ubuntu/.local/share/tg3-teleop/releases/fixture')
        self.assertIn('recording_exporter',command);self.assertNotIn('sudo',command)
        self.assertNotIn('station --',command);self.assertNotIn('token',command)
        with self.assertRaises(ValueError):exporter_command('../wrong')

    def test_real_spawned_writer_process_fails_closed_without_starting_control(self):
        manager=ManualRecording()
        try:
            self.assertIsNone(manager.process)
            with tempfile.TemporaryDirectory() as temp:
                manager.start({'destination':str(Path(temp)/'missing')})
                deadline=time.monotonic()+8
                while manager.poll()['state'] not in ('error','off') and time.monotonic()<deadline:time.sleep(.03)
                self.assertEqual(manager.poll()['state'],'error')
                self.assertTrue(manager.status['incomplete']);self.assertEqual(list(Path(temp).iterdir()),[])
        finally:manager.close()

    @unittest.skipUnless(HAS_MCAP,'PC writer requires desktop/requirements.txt; robot exporter does not require MCAP')
    def test_independent_worker_consumes_binary_fixture_and_finalizes_readable_files(self):
        """Real socket/disk, SSH and ROS source are explicit fixtures (not hardware acceptance)."""
        import queue,socket,threading
        from types import SimpleNamespace as NS
        from mcap.reader import make_reader
        parent,child=socket.socketpair();proceed=threading.Event();ending=threading.Event()
        raw=b'\x00\x01\0\0'+struct.pack('<i',7)
        def produce():
            try:
                for _ in range(32):child.sendall(encode_packet({'kind':'probe'},bytes(65536)))
                child.sendall(encode_packet({'kind':'probe_end','bytes':2097152}))
                if not proceed.wait(5):return
                child.sendall(encode_packet({'kind':'schema','topic':'/n','type':'std_msgs/msg/Int32'},b'int32 data\n'))
                child.sendall(encode_packet({'kind':'ready','topics':{'/n':'std_msgs/msg/Int32'}}))
                child.sendall(encode_packet({'kind':'ros','topic':'/n','timestamp_ns':55},raw))
                child.sendall(encode_packet({'kind':'sample'},b'{"tracking":{"sequence":1}}'))
                ending.wait(5);child.sendall(encode_packet({'kind':'end'}))
            finally:child.close()
        class Channel:
            def settimeout(self,_):parent.settimeout(3)
            def exec_command(self,_):threading.Thread(target=produce,daemon=True).start()
            def sendall(self,data):
                if data==b'proceed\n':proceed.set()
            def recv_stderr(self,_):return b''
            def makefile(self,_):return parent.makefile('rb')
            def shutdown_write(self):ending.set()
            def close(self):parent.close()
        client=NS(get_transport=lambda:NS(open_session=lambda **_:Channel()),close=lambda:None)
        status=queue.Queue();stop=threading.Event()
        with tempfile.TemporaryDirectory() as temp,patch('tg3_pico_dualarm.manual_recording.connect',return_value=client):
            thread=threading.Thread(target=worker,args=({'destination':temp,'remote':'/tmp/fixture',
                'topics':{'/n':'std_msgs/msg/Int32'},'token':'scoped-fixture','rate':1024*1024},stop,status))
            thread.start();deadline=time.monotonic()+5;values=[]
            while time.monotonic()<deadline:
                item=status.get(timeout=5);values.append(item)
                if item['state']=='recording':break
            stop.set();thread.join(10);self.assertFalse(thread.is_alive())
            while not status.empty():values.append(status.get())
            self.assertEqual(values[-1]['state'],'off',values);self.assertTrue(values[-1]['complete'])
            directory=Path(values[-1]['path']);self.assertEqual(json.loads((directory/'session.json').read_text())['state'],'complete')
            with next(directory.glob('*.mcap')).open('rb') as source:messages=list(make_reader(source).iter_messages())
            self.assertEqual(messages[0][2].data,raw)
            self.assertTrue(next(directory.glob('*.jsonl')).read_text())

class RecordingAccessTests(unittest.TestCase):
    def test_recording_input_backlog_stops_only_mirror_and_preserves_live_input(self):
        from test_wireless import native
        key='k'*40;g=InputGateway(role='relay',key=key,clock=lambda:1.)
        token=g.recording_access(key,g.challenge(key)['nonce'],{'protocol':PROTOCOL,'operation':'start'})['token']
        # Disk/network preflight can take seconds: a lease alone does not
        # start a queue that would overflow before the writer is ready.
        for seq in range(150):g.accept(key,g.challenge(key)['nonce'],{'protocol':PROTOCOL,'frame':native(seq)['frame']})
        self.assertEqual(len(g.recording_frames),0);self.assertIsNone(g.recording_fault)
        g.recording_sample(token)
        for seq in range(129):
            g.accept(key,g.challenge(key)['nonce'],{'protocol':PROTOCOL,'frame':native(seq+150)['frame']})
        self.assertEqual(g.tracking.sample().sequence,278)
        with self.assertRaisesRegex(ValueError,'backlog'):g.recording_sample(token)
        g.accept(key,g.challenge(key)['nonce'],{'protocol':PROTOCOL,'frame':native(279)['frame']})
        self.assertEqual(g.tracking.sample().sequence,279)
        g.recording_end(token);self.assertIsNone(g.recording_token)

    def test_scoped_readonly_token_cannot_enable_inject_or_get_control_key(self):
        key='k'*40;gateway=InputGateway(role='relay',key=key)
        calls=[];gateway.control_callback=lambda value:calls.append(value)
        server=WirelessServer(gateway,'127.0.0.1',0).start()
        client=GatewayClient('127.0.0.1',server.address[1],key)
        reader=None
        try:
            scoped=client.recording_access('start')['token'];self.assertNotEqual(scoped,key)
            gateway.runtime={'control':{'fixture_large_feedback':'x'*20000}}
            reader=GatewayClient('127.0.0.1',server.address[1],scoped)
            self.assertIn('tracking',reader.request('GET','/v1/recording/sample'))
            for endpoint in ('/v1/challenge','/v1/status'):
                with self.assertRaises(ValueError):reader.request('GET',endpoint)
            with self.assertRaises(ValueError):reader.control('enable','fixture')
            self.assertEqual(calls,[])
            client.recording_access('stop')
            with self.assertRaises(ValueError):reader.request('GET','/v1/recording/sample')
            self.assertIsNone(gateway.tracking.sample())
        finally:
            if reader:reader.close()
            client.close();server.close()

    def test_record_request_nonce_replay_and_expiry_remain_strict(self):
        clock=[0.];key='k'*40;g=InputGateway(role='relay',key=key,clock=lambda:clock[0])
        nonce=g.challenge(key)['nonce'];clock[0]=.121
        with self.assertRaisesRegex(ValueError,'expired'):g.recording_access(key,nonce,{'protocol':PROTOCOL,'operation':'start'})
        nonce=g.challenge(key)['nonce'];g.recording_access(key,nonce,{'protocol':PROTOCOL,'operation':'start'})
        with self.assertRaisesRegex(ValueError,'expired'):g.recording_access(key,nonce,{'protocol':PROTOCOL,'operation':'stop'})
