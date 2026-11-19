"""Offline fixtures for bounded installed-SDK observations; no native SDK/ROS."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('factory_sdk_evidence',ROOT/'scripts/inspect_factory_readonly.py')
report=importlib.util.module_from_spec(spec);spec.loader.exec_module(report)


class InstalledSdkEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name).resolve();self.sdk=self.base/'xos';self.sdk.mkdir()
        self.site=self.sdk/'robot_control/lib/python3.12/site-packages';self.site.mkdir(parents=True)
        self.proc=self.base/'proc';self.proc.mkdir()

    def write(self,path,text):
        path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text,encoding='utf-8');return path

    def process(self,number,arguments):
        folder=self.proc/str(number);folder.mkdir()
        (folder/'cmdline').write_bytes(b'\0'.join(str(value).encode() for value in arguments)+b'\0')
        # A file that must never be read by the collector.
        (folder/'environ').write_text('TOKEN=private-environment-content',encoding='utf-8')

    def collect(self,**options):
        with patch.object(report.importlib.metadata,'distributions',return_value=[]):
            return report.installed_sdk_metadata(sdk_roots=[self.sdk],proc_root=self.proc,**options)

    def test_metadata_uses_actual_site_packages_without_importing_sdk_or_probe_venv(self):
        module=self.site/'xmigcs_hric_python.cpython-312-aarch64-linux-gnu.so';module.write_bytes(b'native-placeholder')
        distribution=NS(metadata={'Name':'xmigcs'},version='26.06.7',files=[Path(module.name)])
        distribution.locate_file=lambda relative:self.site/relative
        with patch.object(report.importlib.metadata,'distributions',return_value=[distribution]) as distributions, \
             patch.object(report.importlib.metadata,'distribution',side_effect=AssertionError('probe venv lookup')), \
             patch.object(report.importlib,'import_module',side_effect=AssertionError('SDK import')):
            result=report.installed_sdk_metadata(sdk_roots=[self.sdk],proc_root=self.proc)
        distributions.assert_called_once_with(path=[str(self.site.resolve())])
        self.assertEqual(result['distributions'][0]['version'],'26.06.7')
        observed=result['xmigcs_modules'][0]
        self.assertEqual(observed['path'],str(module.resolve()))
        self.assertEqual(observed['distribution_versions'],[{'name':'xmigcs','version':'26.06.7'}])
        self.assertFalse(observed['binary_imported']);self.assertFalse(result['sdk_module_imported'])
        self.assertFalse(result['control_authority_verified'])

    def test_bare_extension_is_listed_without_inventing_a_version(self):
        module=self.site/'xmigcs_hric_python.so';module.write_bytes(b'native')
        result=self.collect()
        self.assertEqual(result['xmigcs_modules'][0]['version_status'],'not_observed')
        self.assertFalse(result['xmigcs_modules'][0]['version_verified'])

    def test_process_paths_and_relevant_configs_are_collected_without_credentials_or_environment(self):
        missing=self.sdk/'robot_control/share/rl_control/xmigcs'
        launch=self.write(self.sdk/'robot_control/launch/robot_control.launch.py',
            f"xmigcs_root = '{missing.as_posix()}'\n# HBWALK uses arms_control_status\n")
        config=self.write(self.sdk/'robot_control/config/stop.yaml',
            'mode: HBWALK\narms_control_status: 0\npassword: launch-password\n'
            'secret: secret-value\nstop: true\n')
        outside=self.write(self.base/'private.yaml','HBWALK: private-outside-content\n')
        self.process(12,['python3',launch,'--config='+str(config),'--password','argv-password',outside])
        self.process(13,['other-service','--token','unrelated-token'])
        result=self.collect();encoded=json.dumps(result)
        observations=result['process_observations']
        self.assertEqual(len(observations['processes']),1)
        self.assertEqual(observations['processes'][0]['sdk_paths'],[str(launch.resolve()),str(config.resolve())])
        self.assertFalse(observations['raw_arguments_included']);self.assertFalse(observations['environments_read'])
        for hidden in ('launch-password','secret-value','argv-password','private-environment-content','private-outside-content','unrelated-token'):
            self.assertNotIn(hidden,encoded)
        sources={Path(item['path']).name:item for item in result['source_config_evidence']}
        self.assertEqual(sources['stop.yaml']['sha256'],hashlib.sha256(config.read_bytes()).hexdigest())
        self.assertTrue(any('arms_control_status' in line['text'] for line in sources['stop.yaml']['matching_snippets']))
        self.assertEqual(result['declared_sdk_paths'][0]['path'],str(missing.resolve()))
        self.assertFalse(result['declared_sdk_paths'][0]['exists'])

    def test_source_excerpts_redact_sensitive_neighbors_and_private_key_blocks(self):
        raw=(b'password: no-output\nHBWALK: yes\n-----BEGIN PRIVATE KEY-----\n'
             b'private-key-body\n-----END PRIVATE KEY-----\narms_control_status: 0\n')
        snippets,_=report._sdk_snippets(raw)
        encoded=json.dumps(snippets)
        self.assertNotIn('no-output',encoded);self.assertNotIn('private-key-body',encoded)
        self.assertIn('REDACTED',encoded)

    def test_source_limits_bound_files_and_bytes_and_omit_full_unrelated_code(self):
        for number in range(4):
            self.write(self.sdk/f'robot_control/source_{number}.py',f'# unrelated-code-{number}\n'*10+'HBWALK = True\n')
        result=self.collect(max_source_files=1,max_total_bytes=512)
        self.assertEqual(result['source_files_inspected'],1);self.assertTrue(result['truncated'])
        self.assertLessEqual(result['bytes_read'],512)
        self.assertLessEqual(len(result['source_config_evidence'][0]['matching_snippets']),3)
        oversized=self.write(self.sdk/'robot_control/config/stop.yaml','HBWALK: '+('x'*1000))
        result=self.collect(max_file_bytes=32)
        evidence=next(item for item in result['source_config_evidence'] if item['path']==str(oversized.resolve()))
        self.assertEqual(evidence['omitted'],'source_exceeds_per_file_limit')
        self.assertNotIn('matching_snippets',evidence)

    def test_outside_roots_and_symlink_escape_are_never_read(self):
        outside=self.write(self.base/'private.py','HBWALK: private-target\n')
        self.assertIsNone(report._sdk_path(outside,(self.sdk,)))
        link=self.sdk/'robot_control/private_alias.py';link.parent.mkdir(exist_ok=True)
        try:link.symlink_to(outside)
        except OSError:return  # Windows without symlink privilege still tests the path boundary above.
        result=self.collect()
        self.assertNotIn('private-target',json.dumps(result))
        self.assertNotIn(str(outside),json.dumps(result))

    def test_process_scan_has_explicit_caps_and_no_raw_argument_output(self):
        for number in range(3):self.process(number+1,['robot_control','--secret','must-not-appear'])
        result=report.sdk_process_evidence((self.sdk,),proc_root=self.proc,max_processes=1,max_bytes=64)
        self.assertEqual(result['cmdlines_read'],1);self.assertTrue(result['truncated'])
        self.assertLessEqual(result['bytes_read'],64)
        self.assertNotIn('must-not-appear',json.dumps(result))

    def test_missing_proc_and_installation_roots_remain_explicit_observations(self):
        with patch.object(report.importlib.metadata,'distributions',return_value=[]):
            result=report.installed_sdk_metadata(sdk_roots=[self.base/'absent-sdk'],proc_root=self.base/'absent-proc')
        self.assertFalse(result['process_observations']['available'])
        self.assertFalse(result['roots'][0]['exists'])
        self.assertEqual(result['distributions'],[])
        self.assertFalse(result['control_authority_verified'])

    def test_xarm_installation_identities_include_package_urdf_plugin_and_schema_without_logs(self):
        xarm=self.base/'XARM';xarm.mkdir()
        self.write(xarm/'install/share/eai_manipulator/package.xml',
            '<package><name>eai_manipulator</name><version>0.2.3</version></package>')
        urdf=self.write(xarm/'install/share/eai_manipulator/robot.urdf','<robot><ros2_control name="EAIHardware"/></robot>')
        self.write(xarm/'install/share/eai_manipulator/plugins.xml','<library><class name="EAIHardware"/></library>')
        schema=self.write(xarm/'install/share/eai_manipulator_msgs/srv/Mode.srv','uint8 mode\n---\nbool success\n')
        self.write(xarm/'logs/private.log','HBWALK private-log-content')
        self.write(xarm/'install/lib/libeai_controller.so','binary-placeholder')
        with patch.object(report.importlib.metadata,'distributions',return_value=[]):
            result=report.installed_sdk_metadata(sdk_roots=[xarm],proc_root=self.proc)
        identities={Path(item['path']).name:item for item in result['installed_file_identities']}
        self.assertEqual(identities['package.xml']['package_name'],'eai_manipulator')
        self.assertEqual(identities['package.xml']['package_version'],'0.2.3')
        self.assertEqual(identities['robot.urdf']['sha256'],hashlib.sha256(urdf.read_bytes()).hexdigest())
        self.assertEqual(identities['Mode.srv']['sha256'],hashlib.sha256(schema.read_bytes()).hexdigest())
        self.assertIn('plugins.xml',identities);self.assertIn('libeai_controller.so',identities)
        self.assertNotIn('private-log-content',json.dumps(result))
        self.assertIsNone(report._sdk_path(xarm/'logs/private.yaml',(xarm,)))
        self.assertIsNone(report._sdk_path(xarm/'install/private.key',(xarm,)))

    def test_xarm_standard_parameter_query_only_reads_selected_names(self):
        class ListParameters:
            class Request:pass
        class GetParameters:
            class Request:pass
        created=[];requests=[];destroyed=[]
        class Client:
            def __init__(self,name):self.name=name
            def wait_for_service(self,**options):return True
            def call_async(self,request):requests.append(request);return request
        class Node:
            def create_client(self,kind,name):created.append(name);return Client(name)
            def destroy_client(self,client):destroyed.append(client)
        consumer=dict(node='jointspace_ruckig_qp_controller',namespace='/',services={
            '/jointspace_ruckig_qp_controller/list_parameters':['rcl_interfaces/srv/ListParameters'],
            '/jointspace_ruckig_qp_controller/get_parameters':['rcl_interfaces/srv/GetParameters'],
            '/EAIHardware/debug':['vendor/srv/Debug']})
        response=NS(result=NS(names=['joint_names','vel_limits','acc_limits','jerk_limits','password','login.token','secret.joint_names']))
        with patch.dict(sys.modules,{'rcl_interfaces':NS(),'rcl_interfaces.srv':NS(ListParameters=ListParameters,GetParameters=GetParameters)}), \
             patch.object(report,'_wait',side_effect=[response,NS(values=[1,2,3,4])]), \
             patch.object(report,'_parameter',side_effect=lambda value:value):
            result=report.read_xarm_parameters(Node(),None,consumer)
        self.assertEqual(set(result['selected']),{'joint_names','vel_limits','acc_limits','jerk_limits'})
        self.assertEqual(len(created),2);self.assertEqual(len(destroyed),2)
        self.assertTrue(all(name.endswith(('/list_parameters','/get_parameters')) for name in created))
        self.assertEqual(requests[1].names,['joint_names','vel_limits','acc_limits','jerk_limits'])

    def test_xarm_graph_records_controller_services_and_dual_consumers_without_calls(self):
        endpoint=NS(node_name='jointspace_ruckig_qp_controller',node_namespace='/',
            topic_type='std_msgs/msg/Float64MultiArray',endpoint_gid=bytes(16))
        class Node:
            def get_topic_names_and_types(self):return [('/jointspace_commands_Dual',['std_msgs/msg/Float64MultiArray'])]
            def get_service_names_and_types(self):return [('/controller_manager/'+name,[kind]) for name,kind in
                [('list_controllers','controller_manager_msgs/srv/ListControllers'),
                 ('switch_controller','controller_manager_msgs/srv/SwitchController'),
                 ('activate_controller','controller_manager_msgs/srv/SwitchController'),
                 ('auto_switch_mode','std_srvs/srv/SetBool')]]+[('/EAIHardware/debug',['vendor/srv/Debug'])]
            def get_node_names_and_namespaces(self):return [('EAIHardware','/'),('controller_manager','/'),('jointspace_ruckig_qp_controller','/')]
            def get_service_names_and_types_by_node(self,*args):return self.get_service_names_and_types()
            def get_publishers_info_by_topic(self,topic):return []
            def get_subscriptions_info_by_topic(self,topic):return [endpoint] if topic=='/jointspace_commands_Dual' else []
            def create_client(self,*args):raise AssertionError('Graph must never call controller services')
        result=report.observe_graph(Node())['xarm_observations']
        self.assertEqual(len(result['services']),5)
        self.assertEqual(result['jointspace_dual_endpoints']['subscribers'][0]['node'],'jointspace_ruckig_qp_controller')
        self.assertEqual(result['services_invoked'],[]);self.assertFalse(result['control_authority_verified'])


if __name__=='__main__':unittest.main()
