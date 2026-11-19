"""Read-only hand observation, independent of a headset or arm controller.

Captures actual six-channel status and observed command fields. A stable
pose is not a mechanical end stop or a proven grasp. Creates no publishers
or service clients and does not change factory settings.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
from .manufacturer_feedback import HandFeedback


def summarize(rows,now,*,tolerance=5,min_count=15):
    if not rows:return dict(ready=False,reason='no_hand_feedback')
    last=rows[-1];fresh=0<=now-last['received_at']<=.25
    minimum=[min(row['positions'][i] for row in rows) for i in range(6)]
    maximum=[max(row['positions'][i] for row in rows) for i in range(6)]
    stable=all(high-low<=tolerance for low,high in zip(minimum,maximum))
    zero=all(all(code==0 for code in row['states']) for row in rows)
    stationary=all(all(speed==0 for speed in row['speeds']) for row in rows)
    sufficient=len(rows)>=min_count and last['received_at']-rows[0]['received_at']>=.5
    ready=fresh and stable and zero and stationary and sufficient
    return dict(ready=ready,fresh=fresh,stable=stable,all_state_codes_zero=zero,stationary=stationary,
        samples=len(rows),sufficient_observation=sufficient,minimum=minimum,maximum=maximum,latest=last,
        reason='stable_observed_pose' if ready else 'feedback_or_stability_not_confirmed',
        mechanical_limits_verified=False,grasp_verified=False)


def run(seconds=5.,*,label='observed',output=None):
    if not .5<=seconds<=60.:raise ValueError('observation_seconds_must_be_0.5_to_60')
    if label not in ('observed','open','pregrasp','grasp','released'):raise ValueError('unknown_observation_label')
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from brainco_hand_msgs.msg import MotorStatus,SetMotorMulti
    context=Context();rclpy.init(args=[],context=context)
    node=Node('tg3_hand_observe_'+str(time.monotonic_ns()),context=context,
        enable_rosout=False,start_parameter_services=False,use_global_arguments=False)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    rows={side:[] for side in ('left','right')};commands={side:[] for side in rows}
    errors={};subscriptions=[]
    def status(side,message):
        try:
            rows[side].append(asdict(HandFeedback.parse(message,time.monotonic())))
            if len(rows[side])>1800:rows[side].pop(0)
        except (ValueError,TypeError) as exc:errors[side]=str(exc)
    def command(side,message):
        row=dict(mode=int(message.mode),received_at=time.monotonic())
        for name in ('positions','speeds','currents'):row[name]=[int(v) for v in getattr(message,name)]
        commands[side].append(row)
        if len(commands[side])>10:commands[side].pop(0)
    try:
        for side in rows:
            for cls,suffix,callback in ((MotorStatus,'motor_status',status),(SetMotorMulti,'set_motor_multi',command)):
                subscriptions.append(node.create_subscription(cls,'/'+side+'_hand/'+suffix,
                    lambda msg,s=side,cb=callback:cb(s,msg),qos_profile_sensor_data))
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        result=dict(passive_only=True,robot_commands_sent=False,headset_required=False,
            label=label,label_source='operator_label_not_automatic_pose_recognition',
            calibration_complete=False,errors=errors,
            hands={side:dict(status=summarize(samples,time.monotonic()),observed_commands=commands[side]) for side,samples in rows.items()})
        if output:
            output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
            with output.open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
        return result
    finally:
        for subscription in subscriptions:node.destroy_subscription(subscription)
        executor.shutdown();node.destroy_node();context.shutdown()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=5.)
    parser.add_argument('--label',default='observed',choices=('observed','open','pregrasp','grasp','released'))
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    print(json.dumps(run(args.seconds,label=args.label,output=args.output),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
