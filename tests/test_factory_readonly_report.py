"""Read-only diagnostic graph/schema collection with explicit non-ROS fixtures."""
import ast
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('factory_readonly_report',ROOT/'scripts/inspect_factory_readonly.py')
report=importlib.util.module_from_spec(spec);spec.loader.exec_module(report)


class FactoryReadonlyReportTests(unittest.TestCase):
    def test_probe_uses_disposable_logs_and_restores_factory_environment_even_on_failure(self):
        for failed in (False,True):
            observed=[]
            def fake_run(seconds):
                path=Path(os.environ['ROS_LOG_DIR']);observed.append(path)
                self.assertTrue(path.is_dir())
                self.assertNotEqual(str(path),'/factory/root-owned/logs')
                (path/'probe.log').write_text('bounded probe fixture')
                if failed:raise RuntimeError('injected probe failure')
                return dict(passive_only=True,hardware_motion_enabled=False)
            with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROS_LOG_DIR':'/factory/root-owned/logs'}),patch.object(report,'run',fake_run),redirect_stdout(io.StringIO()):
                code=report.main(['--seconds','5','--output',str(Path(folder)/'report.json')])
                self.assertEqual(code,2 if failed else 0)
                self.assertEqual(os.environ['ROS_LOG_DIR'],'/factory/root-owned/logs')
                self.assertFalse(observed[0].exists())

    def test_nested_schema_names_include_sequences_fixed_arrays_and_explicit_namespace(self):
        self.assertEqual(report.nested_messages(dict(a='sequence<robot_msgs/Motor>',b='std_msgs/msg/Header',
            c='robot_msgs/Motor[14]',d='float64[6]')),['robot_msgs/msg/Motor','std_msgs/msg/Header'])

    def test_recursive_message_and_service_schemas_survive_a_missing_interface(self):
        class Outer:
            @classmethod
            def get_fields_and_field_types(cls):return {'motor':'sequence<fixture/Motor>'}
        class Motor:
            @classmethod
            def get_fields_and_field_types(cls):return {'position':'float64'}
        class Service:
            Request=Outer;Response=Motor
        def message(name):return {'fixture/msg/Outer':Outer,'fixture/msg/Motor':Motor}[name]
        with patch.object(report,'interface_files',return_value=[]):
            result=report.collect_schemas(['fixture/msg/Outer','missing/msg/Class'],['fixture/srv/Read'],
                get_message=message,get_service=lambda name:Service)
        self.assertEqual(set(result['interfaces']),{'fixture/msg/Outer','fixture/msg/Motor','fixture/srv/Read'})
        self.assertIn('missing/msg/Class',result['errors'])
        self.assertEqual(result['interfaces']['fixture/srv/Read']['definition']['response'],{'position':'float64'})
        self.assertFalse(result['truncated'])

    def test_graph_preserves_actual_executor_and_full_advertised_service_types_without_calls(self):
        endpoint=NS(node_name='actual_factory',node_namespace='/installed',topic_type='ros2_bridge_msgs/msg/ArmCtrl',endpoint_gid=bytes([1]*16))
        class Node:
            def get_topic_names_and_types(self):return [('/arm/cmd',['ros2_bridge_msgs/msg/ArmCtrl']),('/camera/rgb',['sensor_msgs/msg/Image'])]
            def get_service_names_and_types(self):return [('/installed/actual_factory/lease',['factory_interfaces/srv/Lease'])]
            def get_node_names_and_namespaces(self):return [('actual_factory','/installed')]
            def get_publishers_info_by_topic(self,topic):return []
            def get_subscriptions_info_by_topic(self,topic):return [endpoint] if topic=='/arm/cmd' else []
            def get_service_names_and_types_by_node(self,name,namespace):return self.get_service_names_and_types()
            def create_publisher(self,*args):raise AssertionError('Must never create an actuator publisher')
            def create_client(self,*args):raise AssertionError('Graph inspection must never invoke a factory service')
        graph=report.observe_graph(Node())
        self.assertIn('/camera/rgb',graph['topic_types'])
        self.assertEqual(graph['arm_consumers']['/installed/actual_factory']['services'],
            {'/installed/actual_factory/lease':['factory_interfaces/srv/Lease']})

    def test_native_environment_error_is_explicit_and_still_produces_a_readonly_report(self):
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)/'report.json'
            with patch.object(report,'run',side_effect=ImportError('native rclpy missing')),redirect_stdout(io.StringIO()):
                self.assertEqual(report.main(['--output',str(target)]),2)
            value=json.loads(target.read_text(encoding='utf-8'))
        self.assertEqual(value['error'],'native rclpy missing')
        self.assertFalse(value['hardware_motion_enabled']);self.assertEqual(value['motion_publishers_created'],[])

    def test_script_has_no_calls_to_actuator_or_sdk_lifecycle_methods(self):
        tree=ast.parse((ROOT/'scripts/inspect_factory_readonly.py').read_text(encoding='utf-8'))
        forbidden={'create_publisher','request_dual_arm_control','hold','stop_and_restore','release_control',
            'set_parameters','set_parameters_atomically','exec_command','system','Popen'}
        calls={node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)}
        self.assertFalse(calls&forbidden)


if __name__=='__main__':unittest.main()
