"""Measured tool-offset form conversion, with units and an explicit audit note."""
import math
from copy import deepcopy
import numpy as np
from .mapping import rigid_transform


def tcp_reference_tools(base):
    """Define control points, without pretending to measure human/hand offsets."""
    from .model import MODEL_SHA256
    result=deepcopy(base)
    for name in ('controller_to_hand','tcp_to_hand'):
        result[name]={side:np.eye(4).tolist() for side in ('left','right')}
    result.update(tool_reference='urdf_tcp',controller_reference='openxr_grip',
        reference_model_sha256=MODEL_SHA256,tool_offsets_calibrated=False)
    result.pop('measurement_evidence',None)
    return result


def reference_status(config):
    """Validate a defined model reference or a separately measured grasp point.

    Identity here defines OpenXR grip -> URDF TCP relative control. It is not
    a palm measurement, and it does not bypass heading/clutch calibration.
    Legacy configurations retain their previous measured-offset requirement.
    """
    from .model import MODEL_SHA256
    kind=config.get('tool_reference','measured_grasp_point')
    if kind not in ('urdf_tcp','measured_grasp_point'):raise ValueError('unknown_end_effector_reference')
    transforms=[]
    for name in ('controller_to_hand','tcp_to_hand'):
        if set(config.get(name,{}))!={'left','right'}:raise ValueError('both_reference_transforms_required')
        transforms.extend(rigid_transform(config[name][side]) for side in ('left','right'))
    measured=config.get('tool_offsets_calibrated',False)
    if type(measured) is not bool:raise ValueError('explicit_tool_measurement_state_required')
    if kind=='urdf_tcp':
        if (measured or config.get('controller_reference')!='openxr_grip' or
            config.get('reference_model_sha256')!=MODEL_SHA256 or
            any(not np.allclose(t,np.eye(4),rtol=0,atol=1e-10) for t in transforms)):
            raise ValueError('URDF_TCP_requires_verified_model_and_defined_identity_references')
        return dict(kind=kind,ready=True,offsets_measured=False,controller_reference='openxr_grip',
            robot_points={'left':'left_tcp_link','right':'right_tcp_link'},
            description='OpenXR握持点相对控制URDF末端TCP；不声明已测掌心偏移')
    ready=measured and bool(str(config.get('measurement_evidence','')).strip())
    return dict(kind=kind,ready=ready,offsets_measured=measured,
        description='实测抓握点偏移' if ready else '缺少实测抓握点偏移或依据')


def transform_from_mm_degrees(values):
    if len(values)!=6:raise ValueError('需要 XYZ 毫米及 Roll/Pitch/Yaw 角度，共六个数')
    numbers=[float(v) for v in values]
    if any(not math.isfinite(v) for v in numbers):raise ValueError('标定数值必须有限')
    if any(abs(v)>.5*1000 for v in numbers[:3]) or any(abs(v)>180 for v in numbers[3:]):
        raise ValueError('工具位移需在 ±500 毫米内，角度在 ±180 度内')
    r,p,y=[math.radians(v) for v in numbers[3:]]
    rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    rz=np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    result=np.eye(4);result[:3,:3]=rz@ry@rx;result[:3,3]=np.array(numbers[:3])/1000
    return rigid_transform(result).tolist()


def measured_tools(base, entries, evidence, confirmed):
    if confirmed is not True or not isinstance(evidence,str) or not evidence.strip():
        raise ValueError('需要确认实际测量，并填写测量记录')
    if set(entries)!= {'controller_to_hand','tcp_to_hand'}:
        raise ValueError('控制器与机器人工具标定均须填写')
    result=dict(base)
    for name, sides in entries.items():
        if set(sides)!={'left','right'}:raise ValueError('左右需分别标定')
        result[name]={side:transform_from_mm_degrees(values) for side,values in sides.items()}
    result.update(tool_offsets_calibrated=True,tool_reference='measured_grasp_point',measurement_evidence=evidence.strip())
    return result
