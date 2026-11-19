"""Read-only native controller discovery; never call a motion/ownership service.

Reports actual /arm/cmd consumers, their advertised services, installed message
definitions and selected standard ROS parameter values. Endpoint names are
observed, not assumed to confer actuator authority.
"""
import argparse
import importlib
import json
from pathlib import Path
import time
import uuid

from .preflight import _gid

TOPICS=('/arm/cmd','/robot_state','/hric/robot/rl_state','/left_hand/set_motor_multi',
        '/right_hand/set_motor_multi','/left_hand/motor_status','/right_hand/motor_status')
PARAMETER_LEAVES=frozenset(('arms_control_status','control_mode','arm_control_mode','sdk_mode',
    'allow_sdk','arm_control','arm_command_topic','arm_state_topic','emergency_stop_topic','estop_topic','safety_mode'))


def endpoints(values):
    return [dict(node=e.node_name,namespace=e.node_namespace,topic_type=e.topic_type,gid=_gid(e.endpoint_gid)) for e in values]


def graph_snapshot(node):
    subscribers={topic:endpoints(node.get_subscriptions_info_by_topic(topic)) for topic in TOPICS}
    publishers={topic:endpoints(node.get_publishers_info_by_topic(topic)) for topic in TOPICS}
    consumers={}
    for entry in subscribers['/arm/cmd']:
        name,namespace=entry['node'],entry['namespace']
        if not name or name.startswith('_NODE_NAME_UNKNOWN') or not namespace or namespace.startswith('_NODE_NAMESPACE_UNKNOWN'):continue
        key=(namespace.rstrip('/')+'/'+name).replace('//','/')
        services=node.get_service_names_and_types_by_node(name,namespace)
        consumers[key]=dict(node=name,namespace=namespace,services={s:list(types) for s,types in services})
    return dict(passive_only=True,motion_services_called=[],motion_publishers_created=[],
                topics=dict(subscribers=subscribers,publishers=publishers),arm_consumers=consumers,
                dual_arm_control_contract_verified=False,physical_estop_interface_verified=False)


def installed_schemas():
    result={}
    for package,names in (('ros2_bridge_msgs',('ArmCtrl','MotorCtrl','RobotState')),
                           ('brainco_hand_msgs',('SetMotorMulti','MotorStatus'))):
        module=importlib.import_module(package+'.msg')
        for name in names:
            cls=getattr(module,name)
            result[package+'/msg/'+name]=dict(fields=cls.get_fields_and_field_types(),
                python_binding=str(getattr(importlib.import_module(cls.__module__),'__file__','unknown')))
    return result


def _wait(executor,future,timeout):
    deadline=time.monotonic()+timeout
    while not future.done() and time.monotonic()<deadline:
        executor.spin_once(timeout_sec=min(.05,max(0.,deadline-time.monotonic())))
    if not future.done():future.cancel();raise TimeoutError('standard_parameter_request_timeout')
    return future.result()


def _parameter(value):
    fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',
            6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
    field=fields.get(value.type)
    if field is None:return None
    result=getattr(value,field)
    if 'array' in field:return list(result)[:1024]
    return result[:4096] if isinstance(result,str) else result


def read_selected_parameters(node,executor,consumer,timeout=.5):
    from rcl_interfaces.srv import ListParameters,GetParameters
    name=(consumer['namespace'].rstrip('/')+'/'+consumer['node']).replace('//','/')
    advertised=consumer['services']
    list_name=name+'/list_parameters';get_name=name+'/get_parameters'
    result=dict(parameter_names=[],selected={},error=None)
    if advertised.get(list_name)!=['rcl_interfaces/srv/ListParameters'] or advertised.get(get_name)!=['rcl_interfaces/srv/GetParameters']:
        result['error']='standard_parameter_services_not_advertised';return result
    clients=[]
    try:
        listing=node.create_client(ListParameters,list_name);clients.append(listing)
        getting=node.create_client(GetParameters,get_name);clients.append(getting)
        if not listing.wait_for_service(timeout_sec=timeout) or not getting.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('standard_parameter_services_unavailable')
        request=ListParameters.Request();request.prefixes=[];request.depth=0
        response=_wait(executor,listing.call_async(request),timeout)
        names=list(response.result.names)
        result['parameter_names']=names
        selected=[n for n in names if n.rsplit('.',1)[-1] in PARAMETER_LEAVES]
        if selected:
            request=GetParameters.Request();request.names=selected
            values=_wait(executor,getting.call_async(request),timeout).values
            if len(values)!=len(selected):raise RuntimeError('parameter_reply_count_mismatch')
            result['selected']={name:_parameter(value) for name,value in zip(selected,values)}
    except Exception as exc:result['error']=str(exc)
    finally:
        for client in clients:node.destroy_client(client)
    return result


def run(seconds=5.):
    if type(seconds) not in (int,float) or not 1<=seconds<=30:raise ValueError('discovery_seconds_must_be_1_to_30')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    context=Context();rclpy.init(args=[],context=context);node=executor=None
    try:
        node=Node('tg3_manufacturer_readonly_'+uuid.uuid4().hex[:8],context=context,
            enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=min(.1,max(0.,deadline-time.monotonic())))
        result=graph_snapshot(node);result['schemas']=installed_schemas()
        for consumer in result['arm_consumers'].values():
            consumer['parameters']=read_selected_parameters(node,executor,consumer)
        result['own_publishers']=node.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace())
        if any(topic in TOPICS for topic,_ in result['own_publishers']):raise RuntimeError('readonly_discovery_created_motion_publisher')
        return result
    finally:
        if executor:executor.shutdown()
        if node:node.destroy_node()
        if context.ok():context.shutdown()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=5.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    try:result=run(args.seconds)
    except Exception as exc:
        result=dict(passive_only=True,motion_publishers_created=[],error=str(exc),hardware_ready=False)
    data=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)
    if args.output:
        args.output.parent.mkdir(exist_ok=True,parents=True);args.output.write_text(data,encoding='utf-8')
    print(data)
    return 2 if 'error' in result else 0


if __name__=='__main__':raise SystemExit(main())
