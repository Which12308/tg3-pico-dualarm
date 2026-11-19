"""Strict configuration. JSON is used as a dependency-free YAML subset."""
import json
import math
from pathlib import Path


def read_json_yaml(path):
    def reject_constant(value):raise ValueError('nonfinite configuration: '+value)
    def unique(items):
        result={}
        for key,value in items:
            if key in result:raise ValueError('duplicate configuration key: '+key)
            result[key]=value
        return result
    return json.loads(Path(path).read_text(encoding='utf-8'),parse_constant=reject_constant,object_pairs_hook=unique)


def load(directory,*,allow_hardware=False):
    directory=Path(directory)
    result={key:read_json_yaml(directory/(key+'.yaml')) for key in ('vr','robot','calibration','safety')}
    robot=result['robot']
    if (robot['mode'] not in (('dry_run','hardware') if allow_hardware else ('dry_run',))
            or robot['hardware_motion_enabled'] is not False):
        raise ValueError('hardware_mode_requires_verified_manufacturer_dual_arm_adapter')
    if robot['torso_waist_locked'] is not True:
        raise ValueError('waist_motion_is_not_verified_and_is_not_enabled')
    if robot['motor_ids']!=list(range(11,18))+list(range(21,28)):
        raise ValueError('unexpected_joint_order')
    if result['vr']['mode'] not in ('position','pose'):raise ValueError('invalid mapping mode')
    if not .01<=result['vr']['translation_scale']<=1 or not 0<=result['vr']['orientation_scale']<=1:
        raise ValueError('invalid mapping scales')
    for name,value in result['safety'].items():
        if name.endswith(('_s','_rad','_m','_rad_s','_rad_s2','_m_s')):
            if type(value) not in (float,int) or not math.isfinite(value) or value<=0:
                raise ValueError('invalid safety value: '+name)
    # Only named, reviewed session profiles may change the input budget.
    from .input_policy import from_config
    from_config(result['vr'])
    if not 0<result['safety']['workspace_extent_m']<=.2:
        raise ValueError('workspace extension requires separate verification')
    if not 0<result['safety']['watchdog_timeout_s']<=.15:
        raise ValueError('watchdog timeout exceeds the input validity budget')
    from .mapping import Calibration
    options={k:result['calibration'][k] for k in ('controller_to_hand','tcp_to_hand','tool_offsets_calibrated')
             if k in result['calibration']}
    Calibration([[1,0,0],[0,1,0],[0,0,1]],**options)
    from .tool_setup import reference_status
    reference_status(result['calibration'])
    return result
