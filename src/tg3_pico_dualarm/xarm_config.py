"""Strict built-in XARM profile: only documented arm resources and topics."""
import math
from .configuration import read_json_yaml
from .model import DualArmModel


def load_xarm_settings(path):
    from .xarm_adapter import XArmSettings
    value=read_json_yaml(path)
    fields={'schema','controller','topic','velocity_cap_rad_s','feedback_lead_rad','limits_evidence'}
    optional={'feedback_lead_reserve_rad'}
    if (not isinstance(value,dict) or not fields<=set(value)<=fields|optional
            or value['schema']!='tg3_xarm_settings_v1'):
        raise ValueError('invalid_xarm_configuration')
    if (value['controller']!='jointspace_ruckig_qp_controller' or
            value['topic']!='/jointspace_commands_Dual'):
        raise ValueError('only_documented_arm_only_xarm_controller_supported')
    for key,cap in (('velocity_cap_rad_s',.02),('feedback_lead_rad',.03)):
        n=value[key]
        if type(n) not in (int,float) or not math.isfinite(n) or not 0<n<=cap:
            raise ValueError('invalid_xarm_software_cap:'+key)
    reserve=value.get('feedback_lead_reserve_rad',.005)
    # The reserve only narrows the target window inside the fixed submit
    # check; it can never widen feedback_lead_rad.
    if (type(reserve) not in (int,float) or not math.isfinite(reserve) or not 0<=reserve<=.01
            or reserve>=value['feedback_lead_rad']):
        raise ValueError('invalid_xarm_software_cap:feedback_lead_reserve_rad')
    if not isinstance(value['limits_evidence'],str) or not value['limits_evidence'].strip():
        raise ValueError('xarm_limit_evidence_required')
    model=DualArmModel()
    return XArmSettings(velocity_cap=(value['velocity_cap_rad_s'],)*14,
        lower=tuple(model.limits[:,0]),upper=tuple(model.limits[:,1]),
        feedback_lead=(value['feedback_lead_rad'],)*14,feedback_lead_reserve=reserve,
        limits_evidence=value['limits_evidence'],controller=value['controller'],topic=value['topic'])
