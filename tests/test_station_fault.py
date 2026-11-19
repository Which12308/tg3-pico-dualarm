"""Real HTTP fault-state checks; explicit controller double never controls a robot."""
import threading
from types import SimpleNamespace as NS
import unittest

from tg3_pico_dualarm.station_fault import latch_fault
from tg3_pico_dualarm.wireless import InputGateway,WirelessServer,GatewayClient,PROTOCOL
from test_wireless import native


class StationFaultTests(unittest.TestCase):
    def test_diagnostics_stay_available_but_new_input_and_enable_cannot_resume_control(self):
        calls=[];key='TEST ONLY KEY '*4
        gateway=InputGateway(role='relay',key=key)
        gateway.runtime={'readiness':{'hardware_motion_enabled':True,'blockers':[]}}
        controller=NS(disarm=lambda reason:(calls.append(reason) or dict(
            manufacturer_stop_acknowledged=False,robot_commands_sent=False)))
        with_status={'solve_s':.25}
        fault=latch_fault(gateway,controller,'native_ik_worker_fault',details=with_status)
        with_status['solve_s']=0.
        self.assertEqual(fault['failure_details']['solve_s'],.25)
        self.assertFalse(fault['manufacturer_stop_acknowledged'])
        server=WirelessServer(gateway,'127.0.0.1',0).start()
        client=GatewayClient(*server.address,token=key,timeout=1.)
        try:
            status=client.request('GET','/v1/status')
            self.assertEqual(status['runtime']['control']['state'],'fault')
            self.assertFalse(status['hardware_motion_enabled'])
            self.assertFalse(status['runtime']['readiness']['ready_to_request_motion'])
            result=client.control('enable','new-source')
            self.assertFalse(result['accepted']);self.assertEqual(result['reason'],'station_fault_restart_required')
            client.send(dict(protocol=PROTOCOL,frame=native()['frame']))
            self.assertEqual(client.request('GET','/v1/status')['runtime']['control']['state'],'fault')
            self.assertEqual(calls,['station_fault:native_ik_worker_fault'])
        finally:client.close();server.close()
