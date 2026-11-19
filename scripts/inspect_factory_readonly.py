"""Read installed factory interfaces, graph and feedback; never command hardware.

Run on the actual Ubuntu after sourcing ~/xos/setup.bash. No ownership, mode,
hold, stop, restore or E-stop service is invoked. Standard List/GetParameters
requests are the only service calls. No vendor SDK module is imported.
"""
import argparse
from collections import deque
from datetime import datetime,timezone
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import sys
import tempfile
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from tg3_pico_dualarm.manufacturer_discovery import TOPICS,endpoints,read_selected_parameters
from tg3_pico_dualarm.manufacturer_feedback import ManufacturerFeedback
from tg3_pico_dualarm.manufacturer_discovery import _wait,_parameter

FIXED_MESSAGES=('ros2_bridge_msgs/msg/ArmCtrl','ros2_bridge_msgs/msg/MotorCtrl',
    'ros2_bridge_msgs/msg/RobotState','brainco_hand_msgs/msg/SetMotorMulti','brainco_hand_msgs/msg/MotorStatus',
    'bodyctrl_msgs/msg/PowerBoardKeyStatus')
INTERFACE_WORDS=re.compile(r'arm|hand|estop|e_stop|emergency|safety|ownership|arbitrat|controller|fsm|rl_state',re.I)
FEEDBACK_WORDS=re.compile(r'estop|e_stop|emergency|safety|ownership|owner|control.*(?:state|status)|(?:state|status).*control',re.I)
REQUIRED_CONTRACTS=(
    'Installed controller identity/version and authoritative interface semantics',
    'Observed dual-arm ownership acknowledgement covering motor IDs 11..17 and 21..27 and selected hands',
    'Independent physical E-stop observation with documented polarity and freshness',
    'Manufacturer hold mechanism and its acknowledgement',
    'Manufacturer stop/restore and release mechanisms, acknowledgement and recovery state',
    'Actual per-joint gains/units/limits and per-hand calibration/current limits')

SDK_ROOTS=('/home/ubuntu/xos','/opt/PARTITIONS/A/xos',
    '/home/ubuntu/.local/lib/python3.12/site-packages','/home/ubuntu/XARM')
SDK_IDENTITY=re.compile(r'xmigcs|xhumanoid[-_]?sdk|robot[-_]control|rl[-_]control|xarm|eai_manipulator',re.I)
SDK_EVIDENCE=re.compile(r'HBWALK|arms_control_status|xmigcs_root|stop\.ya?ml|site-packages|sys\.path|handoff|handover|'
    r'ownership|release_control|request_dual_arm_control|arm\w*.*(?:enable|disable|control)|'
    r'(?:mode|state).*transition|jointspace_ruckig|jointspace_commands_Dual|auto_switch|empty_[LR]_controller|'
    r'joint_names|vel_limits|acc_limits|jerk_limits|ros2_control|hardware_interface|plugin|<version>|<name>|<depend',re.I)
SENSITIVE_CONTENT=re.compile(r'password|passwd|secret|token|api[_-]?key|private[_-]?key|credential|authorization|login',re.I)
XARM_PARAMETER_LEAVES=frozenset(('joint_names','vel_limits','acc_limits','jerk_limits',
    'arms_control_status','arm_enable','arms_enabled','arm_enabled','arm_mode','control_mode','hardware_mode',
    'is_real','hardware','enabled','state','status','auto_switch_mode'))


def nested_messages(fields):
    result=set()
    for field_type in fields.values():
        for package,namespace,name in re.findall(r'\b([A-Za-z_]\w*)/(?:(msg)/)?([A-Za-z_]\w*)\b',str(field_type)):
            result.add(package+'/msg/'+name)
    return sorted(result)


def interface_files(name):
    from ament_index_python.packages import get_package_share_directory
    package,kind,leaf=name.split('/')
    share=Path(get_package_share_directory(package)).resolve();result=[]
    for suffix in ('.msg','.srv','.idl'):
        path=share/kind/(leaf+suffix)
        if not path.is_file():continue
        resolved=path.resolve()
        if share not in resolved.parents:continue
        raw=resolved.read_bytes()
        result.append(dict(path=str(resolved),sha256=hashlib.sha256(raw).hexdigest(),
            text=raw[:65536].decode('utf-8',errors='replace'),truncated=len(raw)>65536))
    return result


def collect_schemas(messages,services,*,get_message=None,get_service=None,max_types=1024):
    if get_message is None or get_service is None:
        from rosidl_runtime_py.utilities import get_message,get_service
    pending=deque((name,'message') for name in sorted(set(messages)))
    pending.extend((name,'service') for name in sorted(set(services)))
    result={};errors={}
    while pending and len(result)+len(errors)<max_types:
        name,kind=pending.popleft()
        if name in result or name in errors:continue
        try:
            cls=(get_message if kind=='message' else get_service)(name)
            children=[]
            if kind=='message':
                fields=dict(cls.get_fields_and_field_types());definition=dict(fields=fields)
                children+=nested_messages(fields)
            else:
                request=dict(cls.Request.get_fields_and_field_types());response=dict(cls.Response.get_fields_and_field_types())
                definition=dict(request=request,response=response)
                children+=nested_messages(request)+nested_messages(response)
            binding=getattr(importlib.import_module(cls.__module__),'__file__',None)
            files=[];file_error=None
            try:files=interface_files(name)
            except Exception as exc:file_error=str(exc)
            result[name]=dict(kind=kind,python_binding=binding,definition=definition,installed_interface_files=files,
                interface_file_error=file_error)
            pending.extend((child,'message') for child in children)
        except Exception as exc:errors[name]=str(exc)
    return dict(interfaces=result,errors=errors,truncated=bool(pending),unprocessed_count=len(pending))


def _sdk_path(path,roots):
    """Resolve symlinks before accepting a path inside an explicit SDK root."""
    try:
        resolved=Path(path).resolve()
        allowed=next((root for root in roots if resolved==root or root in resolved.parents),None)
        if allowed is None:return None
        parts=resolved.relative_to(allowed).parts
        denied={'logs','log','keys','certs','recordings','bags','models','model','meshes','mesh'}
        if any(part.lower() in denied or SENSITIVE_CONTENT.search(part) for part in parts):return None
        if resolved.suffix.lower() in ('.pem','.key','.p12','.pfx','.log','.bag','.db3','.mcap'):return None
        return resolved
    except (OSError,RuntimeError,ValueError):return None


def sdk_process_evidence(roots,*,proc_root=Path('/proc'),max_processes=1024,max_bytes=4*1024*1024):
    """Read cmdline only; report SDK paths, never raw argv or environments."""
    result=dict(processes=[],cmdlines_read=0,bytes_read=0,truncated=False,
        raw_arguments_included=False,environments_read=False,control_authority_verified=False)
    try:entries=os.scandir(proc_root)
    except OSError:return dict(result,available=False)
    for entry in entries:
        if not entry.name.isdigit():continue
        entry=Path(entry.path)
        if result['cmdlines_read']>=max_processes or result['bytes_read']>=max_bytes:
            result['truncated']=True;break
        try:
            with (entry/'cmdline').open('rb') as stream:raw=stream.read(min(16384,max_bytes-result['bytes_read']))
        except OSError:continue
        result['cmdlines_read']+=1;result['bytes_read']+=len(raw)
        if len(raw)>=16384 and not raw.endswith(b'\0'):
            result['truncated']=True;continue
        arguments=[value.decode('utf-8',errors='replace') for value in raw.split(b'\0') if value]
        if not any(SDK_IDENTITY.search(argument) for argument in arguments):continue
        paths=[];omit_next=False
        for argument in arguments:
            if omit_next:omit_next=False;continue
            if SENSITIVE_CONTENT.search(argument):
                omit_next=argument.startswith('-') and '=' not in argument
                continue
            value=argument.split('=',1)[1] if argument.startswith('-') and '=' in argument else argument
            if not value:continue
            candidate=Path(value)
            if not candidate.is_absolute():
                # Relative launch/config files can be resolved using cwd, but
                # their contents are still constrained to the SDK roots.
                if candidate.suffix.lower() not in ('.py','.yaml','.yml','.json'):continue
                try:candidate=(entry/'cwd').resolve(strict=True)/candidate
                except (OSError,RuntimeError):continue
            path=_sdk_path(candidate,roots)
            if path is not None and str(path) not in paths:paths.append(str(path))
        result['processes'].append(dict(pid=int(entry.name),argument_count=len(arguments),
            sdk_paths=paths,other_arguments_omitted=True))
    entries.close()
    return dict(result,available=True)


def _sdk_inventory(roots,*,max_entries=12000,max_directories=1500):
    pending=deque(roots);visited=set();files=[];sites=set();entries_seen=0;truncated=False
    while pending:
        directory=_sdk_path(pending.popleft(),roots)
        if directory is None or directory in visited:continue
        if len(visited)>=max_directories:truncated=True;break
        visited.add(directory)
        if directory.name in ('site-packages','dist-packages'):sites.add(directory)
        try:entries=os.scandir(directory)
        except OSError:continue
        with entries:
            for entry in entries:
                if entries_seen>=max_entries:truncated=True;break
                entries_seen+=1
                if entry.name.startswith('.') or SENSITIVE_CONTENT.search(entry.name):continue
                path=_sdk_path(entry.path,roots)
                if path is None:continue
                # Resolved targets must already pass _sdk_path; visited paths
                # prevent cycles while allowing normal ROS installation links.
                try:is_directory=path.is_dir();is_file=path.is_file()
                except OSError:continue
                if is_directory:
                    if entry.name.lower() in ('__pycache__','models','model','assets','node_modules','logs','log',
                                             'meshes','mesh','bags','recordings','keys','certs'):continue
                    if directory.name in ('site-packages','dist-packages') and not SDK_IDENTITY.search(entry.name):continue
                    pending.append(path)
                elif is_file:
                    if SDK_IDENTITY.search(str(path)) or entry.name.lower()=='stop.yaml':files.append(path)
        if truncated:break
    return dict(files=files,site_packages=sorted(sites),entries_seen=entries_seen,
        directories_seen=len(visited),truncated=truncated)


def _sdk_snippets(raw,*,max_lines=60,max_characters=8192):
    lines=raw.decode('utf-8',errors='replace').splitlines();safe=[];private_block=False
    for line in lines:
        if re.search(r'-----BEGIN .*PRIVATE KEY-----',line):private_block=True
        hidden=private_block or bool(SENSITIVE_CONTENT.search(line))
        safe.append('[REDACTED: sensitive content]' if hidden else line[:500])
        if re.search(r'-----END .*PRIVATE KEY-----',line):private_block=False
    selected=set()
    for index,line in enumerate(safe):
        if SDK_EVIDENCE.search(line):
            selected.update(range(max(0,index-1),min(len(safe),index+2)))
    snippets=[];characters=0
    for index in sorted(selected):
        if len(snippets)>=max_lines or characters+len(safe[index])>max_characters:break
        snippets.append(dict(line=index+1,text=safe[index]));characters+=len(safe[index])
    return snippets,len(snippets)<len(selected)


def installed_sdk_metadata(*,sdk_roots=None,proc_root=Path('/proc'),max_source_files=120,
                           max_file_bytes=262144,max_total_bytes=4*1024*1024):
    """Bounded installation/source observations; never import or execute SDK.

    Metadata is searched in installed SDK site-packages, not this probe's venv.
    Explicit roots/proc_root arguments support offline fixtures only; the CLI
    always uses the fixed manufacturer roots above.
    """
    roots=tuple(Path(path).absolute() for path in (SDK_ROOTS if sdk_roots is None else sdk_roots))
    process=sdk_process_evidence(roots,proc_root=proc_root)
    inventory=_sdk_inventory(roots);distributions=[];module_versions={};errors=[]
    for site in inventory['site_packages']:
        try:
            for index,distribution in enumerate(importlib.metadata.distributions(path=[str(site)])):
                if index>=1000:errors.append('distribution_limit_reached');break
                name=distribution.metadata.get('Name','')
                if not SDK_IDENTITY.search(name):continue
                base=_sdk_path(distribution.locate_file(''),roots)
                if base is None:continue
                distributions.append(dict(name=name,version=distribution.version,installation_root=str(base),
                    metadata_search_path=str(site),sdk_module_imported=False))
                for file_index,relative in enumerate(distribution.files or ()):
                    if file_index>=1000:errors.append('distribution_file_limit_reached');break
                    if not str(relative).endswith('.so') or 'xmigcs' not in str(relative).lower():continue
                    module_path=_sdk_path(distribution.locate_file(relative),roots)
                    if module_path is not None:
                        module_versions.setdefault(str(module_path),[]).append(dict(name=name,version=distribution.version))
        except (OSError,ValueError,importlib.metadata.PackageNotFoundError):errors.append('distribution_metadata_unavailable')
    candidates=[]
    for observation in process['processes']:
        candidates.extend(Path(path) for path in observation['sdk_paths'])
    # Prioritize known handover configs before general module source files.
    candidates.extend(sorted(inventory['files'],key=lambda path:(path.name.lower()!='stop.yaml',
        path.suffix.lower() not in ('.yaml','.yml'),path.name!='package.xml',
        path.suffix.lower() not in ('.msg','.srv','.idl','.urdf','.xacro'),str(path))))
    sources=[];modules=[];installed_identities=[];declared_paths=[];seen=set();bytes_read=0;files_inspected=0;truncated=False
    source_suffixes={'.py','.pyi','.h','.hpp','.c','.cpp','.yaml','.yml','.json','.xml','.toml','.ini','.cfg',
        '.urdf','.xacro','.msg','.srv','.idl','.bash'}
    for candidate in candidates:
        path=_sdk_path(candidate,roots)
        if path is None or path in seen or not path.is_file():continue
        seen.add(path)
        try:size=path.stat().st_size
        except OSError:continue
        if path.suffix.lower()=='.so' and 'xmigcs' in path.name.lower():
            versions=module_versions.get(str(path),[])
            if len(modules)<100:modules.append(dict(path=str(path),bytes=size,distribution_versions=versions,
                version_verified=bool(versions),version_status=('distribution_record' if versions else 'not_observed'),binary_imported=False))
            else:truncated=True
            continue
        if path.suffix.lower()=='.so' and 'xarm' in str(path).lower():
            if len(installed_identities)<100:
                installed_identities.append(dict(path=str(path),bytes=size,file_kind='installed_shared_library',binary_imported=False))
            else:truncated=True
            continue
        if path.suffix.lower() not in source_suffixes:continue
        if files_inspected>=max_source_files or bytes_read+min(size,max_file_bytes+1)>max_total_bytes:
            truncated=True;continue
        files_inspected+=1
        info=dict(path=str(path),bytes=size,interpretation='source_observation_not_control_authority')
        if size>max_file_bytes:
            info['omitted']='source_exceeds_per_file_limit';sources.append(info);continue
        try:
            with path.open('rb') as stream:raw=stream.read(min(max_file_bytes+1,max_total_bytes-bytes_read))
        except OSError:continue
        bytes_read+=len(raw)
        if len(raw)>max_file_bytes:
            info['omitted']='source_grew_beyond_per_file_limit';sources.append(info);continue
        if len(raw)!=size:
            info['omitted']='source_size_changed_during_read';sources.append(info);truncated=True;continue
        info['sha256']=hashlib.sha256(raw).hexdigest()
        if path.name=='package.xml' or path.suffix.lower() in ('.urdf','.xacro','.msg','.srv','.idl') or 'plugin' in path.name.lower():
            identity=dict(path=str(path),bytes=size,sha256=info['sha256'],file_kind=path.suffix.lower().lstrip('.'))
            if path.name=='package.xml':
                text=raw.decode('utf-8',errors='replace')
                identity['package_name']=next(iter(re.findall(r'<name>\s*([\w.-]+)\s*</name>',text)),None)
                identity['package_version']=next(iter(re.findall(r'<version[^>]*>\s*([\w.+-]+)\s*</version>',text)),None)
            if len(installed_identities)<100:installed_identities.append(identity)
            else:truncated=True
        info['matching_snippets'],info['snippets_truncated']=_sdk_snippets(raw)
        if path.suffix.lower() in ('.msg','.srv','.idl'):
            # Message/service declarations are interface evidence. Preserve a
            # bounded schema, not implementation code or arbitrary metadata.
            schema_lines=raw.decode('utf-8',errors='replace').splitlines()
            info['schema_lines']=[dict(line=index+1,text=('[REDACTED: sensitive content]' if SENSITIVE_CONTENT.search(line) else line[:500]))
                for index,line in enumerate(schema_lines[:60])]
            info['schema_truncated']=len(schema_lines)>60
        # A configured SDK root can be absent; record that mismatch explicitly.
        for match in re.finditer(r'\bxmigcs_root\s*[:=]\s*[\"\']?([^\"\'\s,#)]+)',raw.decode('utf-8',errors='replace')):
            target=_sdk_path(match.group(1),roots)
            if target is not None and len(declared_paths)<100:
                declared_paths.append(dict(setting='xmigcs_root',path=str(target),exists=target.exists(),declared_by=str(path)))
        if info['matching_snippets'] or path.name.lower()=='stop.yaml' or path.suffix.lower() in ('.msg','.srv','.idl'):sources.append(info)
    return dict(evidence_type='bounded_installed_sdk_observation',control_authority_verified=False,
        sdk_module_imported=False,sdk_constructors_called=False,roots=[dict(path=str(root),exists=root.exists()) for root in roots],
        metadata_search_paths=[str(path) for path in inventory['site_packages']],distributions=distributions,
        xmigcs_modules=modules,installed_file_identities=installed_identities,
        process_observations=process,source_config_evidence=sources,
        declared_sdk_paths=declared_paths,errors=errors,
        limits=dict(max_source_files=max_source_files,max_file_bytes=max_file_bytes,max_total_bytes=max_total_bytes,
            max_inventory_entries=12000,max_inventory_directories=1500),
        bytes_read=bytes_read,source_files_inspected=files_inspected,inventory_entries_seen=inventory['entries_seen'],
        truncated=truncated or inventory['truncated'] or process['truncated'],
        note='Paths, versions and source/config excerpts are observations; HBWALK or arms_control_status does not confirm control authority.')


def observe_graph(node):
    topic_types={name:list(types) for name,types in node.get_topic_names_and_types()}
    service_types={name:list(types) for name,types in node.get_service_names_and_types()}
    topics={};consumers={}
    selected=set(TOPICS)|{'/power/board/key_status','/jointspace_commands_Dual','/manipulator/warn_and_error_info'}|{name for name in topic_types if INTERFACE_WORDS.search(name)}
    for topic in sorted(selected):
        try:
            topics[topic]=dict(publishers=endpoints(node.get_publishers_info_by_topic(topic)),
                subscribers=endpoints(node.get_subscriptions_info_by_topic(topic)))
        except Exception as exc:topics[topic]=dict(error=str(exc))
    for endpoint in topics.get('/arm/cmd',{}).get('subscribers',[]):
        name,namespace=endpoint['node'],endpoint['namespace']
        if not name or not namespace or 'UNKNOWN' in name or 'UNKNOWN' in namespace:continue
        key=(namespace.rstrip('/')+'/'+name).replace('//','/')
        try:services={name:list(types) for name,types in node.get_service_names_and_types_by_node(name,namespace)}
        except Exception as exc:
            consumers[key]=dict(node=name,namespace=namespace,error=str(exc));continue
        consumers[key]=dict(node=name,namespace=namespace,services=services)
    nodes=[dict(node=name,namespace=namespace) for name,namespace in node.get_node_names_and_namespaces()]
    xarm_nodes={}
    for entry in nodes:
        if entry['node'] not in ('EAIHardware','controller_manager','jointspace_ruckig_qp_controller'):continue
        key=(entry['namespace'].rstrip('/')+'/'+entry['node']).replace('//','/')
        try:advertised={name:list(types) for name,types in node.get_service_names_and_types_by_node(entry['node'],entry['namespace'])}
        except Exception:advertised={}
        xarm_nodes[key]=dict(**entry,services=advertised)
    xarm_services={name:types for name,types in service_types.items() if name.startswith('/EAIHardware/') or
        (name.startswith('/controller_manager/') and re.search(r'list|switch|activate',name,re.I))}
    return dict(topic_types=topic_types,service_types=service_types,nodes=nodes,
        relevant_endpoints=topics,arm_consumers=consumers,
        xarm_observations=dict(evidence_type='graph_observation_not_control_authority',control_authority_verified=False,
            nodes=xarm_nodes,services=xarm_services,jointspace_dual_endpoints=topics['/jointspace_commands_Dual'],
            services_invoked=[],controller_state_not_queried=True))


def read_xarm_parameters(node,executor,consumer,timeout=.5):
    """Only standard List/GetParameters; never invoke vendor debug services."""
    from rcl_interfaces.srv import ListParameters,GetParameters
    name=(consumer['namespace'].rstrip('/')+'/'+consumer['node']).replace('//','/')
    advertised=consumer['services'];listing_name=name+'/list_parameters';getting_name=name+'/get_parameters'
    result=dict(selected={},error=None,standard_parameter_services_only=True,control_authority_verified=False)
    if advertised.get(listing_name)!=['rcl_interfaces/srv/ListParameters'] or advertised.get(getting_name)!=['rcl_interfaces/srv/GetParameters']:
        result['error']='standard_parameter_services_not_advertised';return result
    clients=[]
    try:
        listing=node.create_client(ListParameters,listing_name);clients.append(listing)
        getting=node.create_client(GetParameters,getting_name);clients.append(getting)
        if not listing.wait_for_service(timeout_sec=timeout) or not getting.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('standard_parameter_services_unavailable')
        request=ListParameters.Request();request.prefixes=[];request.depth=0
        response=_wait(executor,listing.call_async(request),timeout)
        names=list(response.result.names)[:4096]
        selected=[value for value in names if value.rsplit('.',1)[-1] in XARM_PARAMETER_LEAVES and not SENSITIVE_CONTENT.search(value)][:64]
        if selected:
            request=GetParameters.Request();request.names=selected
            values=_wait(executor,getting.call_async(request),timeout).values
            if len(values)!=len(selected):raise RuntimeError('parameter_reply_count_mismatch')
            result['selected']={name:_parameter(value) for name,value in zip(selected,values)}
    except Exception:result['error']='standard_parameter_observation_failed'
    finally:
        for client in clients:node.destroy_client(client)
    return result


def run(seconds=8.):
    if type(seconds) not in (int,float) or not 5<=seconds<=30:raise ValueError('seconds_must_be_5_to_30')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    context=Context();node=executor=None;observer=None
    rclpy.init(args=[],context=context);started=time.monotonic()
    def spin_for(duration):
        deadline=time.monotonic()+duration
        while context.ok() and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=min(.05,max(0.,deadline-time.monotonic())))
    try:
        node=Node('tg3_factory_evidence_readonly_'+uuid.uuid4().hex[:8],context=context,
            enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        spin_for(2.)
        graph=observe_graph(node);errors=[]
        try:observer=ManufacturerFeedback().subscribe(node)
        except Exception as exc:errors.append('parsed_feedback_subscription:'+str(exc))
        samples={};subscriptions=[]
        chosen={name:types for name,types in graph['topic_types'].items() if name in
            ('/robot_state','/hric/robot/rl_state','/left_hand/motor_status','/right_hand/motor_status',
             '/power/board/key_status') or FEEDBACK_WORDS.search(name)}
        for topic,types in sorted(chosen.items()):
            if len(types)!=1:
                samples[topic]=dict(error='ambiguous_topic_types',types=types);continue
            try:
                cls=get_message(types[0]);samples[topic]=dict(type=types[0],count=0,last=None,interpretation='uninterpreted_observation')
                def callback(message,topic=topic):
                    sample=samples[topic];sample['count']+=1;sample['received_at_monotonic']=time.monotonic()
                    sample['last']=None
                    try:
                        value=message_to_ordereddict(message)
                        encoded=json.dumps(value,ensure_ascii=False,default=list,allow_nan=False)
                        sample['last']=json.loads(encoded) if len(encoded)<=131072 else dict(omitted='sample_exceeded_128KiB')
                        sample.pop('last_decode_error',None)
                    except Exception as exc:sample['last_decode_error']=str(exc)
                subscriptions.append(node.create_subscription(cls,topic,callback,qos_profile_sensor_data))
            except Exception as exc:samples[topic]=dict(type=types[0],error=str(exc))
        spin_for(max(0.,seconds-(time.monotonic()-started)))
        captured=time.monotonic();graph=observe_graph(node)
        feedback=observer.snapshot(require_hands=False) if observer is not None else None
        for consumer in graph['arm_consumers'].values():
            if 'services' in consumer:consumer['selected_standard_parameters']=read_selected_parameters(node,executor,consumer,timeout=.5)
        for consumer in graph['xarm_observations']['nodes'].values():
            consumer['selected_standard_parameters']=read_xarm_parameters(node,executor,consumer,timeout=.5)
        messages=set(FIXED_MESSAGES)
        for types in graph['topic_types'].values():messages.update(types)
        services={typ for types in graph['service_types'].values() for typ in types}
        own=node.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace())
        unexpected=[name for name,_ in own if name!='/parameter_events']
        if unexpected:raise RuntimeError('readonly_probe_unexpected_publishers:'+repr(unexpected))
        try:ros_client_version=importlib.metadata.version('rclpy')
        except importlib.metadata.PackageNotFoundError:ros_client_version=None
        return dict(passive_only=True,hardware_motion_enabled=False,robot_commands_sent=False,
            generated_at_utc=datetime.now(timezone.utc).isoformat(),system=dict(platform=platform.platform(),python=sys.version),
            ros_runtime_observation=dict(rclpy_module_path=str(getattr(rclpy,'__file__','unknown')),
                rclpy_distribution_version=ros_client_version,environment_variables_read=False,
                note='Module path/version observation only; no ROS_DISTRO environment read or SDK import.'),
            duration_s=time.monotonic()-started,feedback_captured_at_monotonic=captured,
            graph=graph,schemas=collect_schemas(messages,services),parsed_feedback=feedback,
            raw_feedback_observations=samples,subscription_errors=errors,own_publishers=own,
            motion_publishers_created=[],motion_services_called=[],vendor_sdk_modules_imported=[],
            standard_parameter_service_calls_only=True,installed_sdk_metadata=installed_sdk_metadata(),
            provider_review=dict(verified=False,motion_authorized=False,
                required_contracts_for_manual_evidence_review=list(REQUIRED_CONTRACTS),
                note='Topic names, schemas, raw zero state codes and running mode do not establish control authority or E-stop polarity.'))
    finally:
        if observer is not None:observer.close()
        if executor is not None:executor.shutdown()
        if node is not None:node.destroy_node()
        if context.ok():context.shutdown()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=8.)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    try:
        # XOS may point ROS_LOG_DIR at root-owned factory logs. This probe
        # runs as the login user and must not chmod/chown factory directories.
        # Disposable logs are not trajectories and never survive this probe.
        previous=os.environ.get('ROS_LOG_DIR')
        try:
            with tempfile.TemporaryDirectory(prefix='tg3-readonly-ros-') as logs:
                os.environ['ROS_LOG_DIR']=logs
                result=run(args.seconds);code=0
        finally:
            if previous is None:os.environ.pop('ROS_LOG_DIR',None)
            else:os.environ['ROS_LOG_DIR']=previous
    except Exception as exc:
        result=dict(passive_only=True,hardware_motion_enabled=False,robot_commands_sent=False,
            motion_publishers_created=[],motion_services_called=[],error=str(exc));code=2
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(passive_only=True,report=str(args.output.resolve()),error=result.get('error'),
        hardware_motion_enabled=False,graph_topics=len(result.get('graph',{}).get('topic_types',{})),
        arm_consumers=list(result.get('graph',{}).get('arm_consumers',{}))),ensure_ascii=False),flush=True)
    return code


if __name__=='__main__':raise SystemExit(main())
