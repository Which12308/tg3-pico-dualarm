"""Reviewed, installed DEX holding gains; never change vendor configuration.

Evidence: passive readback 2026-11-18. Both STOP and STANDZERO declare the
same 31-motor order (12 legs, 3 waist, 14 arms, 2 head). Only arms are used.
These are not the different gains from the archived joystick implementation.
"""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

ROOT=Path('/home/ubuntu/.local/lib/python3.12/site-packages/xmigcs')
SOURCES={
    'policy/stop/config/stop.yaml':'0317f16a3eeb9a9de66be5f34be62b6beb554432fb530a2dfb332d8d0158212c',
    'policy/standzero/config/standzero.yaml':'d24467e97cfe89ddb42bf9ee46bbb72a7f6640026046c5f529f364ffecd68a32',
    'policy/hbwalk/config/hbwalk_lab.yaml':'cc1ccc6d4e20ed5c0cba89ee407e10da796b87b25df5e4c3f5175dfa3901273a',
    'config/dex_config.yaml':'4e98e047f1bbf50ed7909f0890918f63a08c62eba23eca8790541fb223c2bd3c',
}


@dataclass(frozen=True)
class HoldingProfile:
    kp: tuple
    kd: tuple
    sources: tuple
    arm_mode: int=0  # EAI service 0 -> ArmCtrl hybrid 1; enums differ.

    def __post_init__(self):
        if self.arm_mode!=0:raise ValueError('only_reviewed_hybrid_holding_profile_supported')
        for name,maximum in (('kp',2000.),('kd',300.)):
            values=getattr(self,name)
            if len(values)!=14 or any(type(v) not in (int,float) or not math.isfinite(v) or not 0<v<=maximum for v in values):
                raise ValueError('invalid_installed_holding_'+name)

    def status(self):
        return dict(arm_service_mode=0,armctrl_mode=1,kp=list(self.kp),kd=list(self.kd),sources=dict(self.sources))

    def parameter_values(self,response):
        values=getattr(response,'values',())
        if len(values)!=2:raise RuntimeError('factory_holding_parameters_missing')
        result={}
        for key,item in zip(('kp','kd'),values):
            vector=tuple(item.double_array_value)
            if item.type!=8 or len(vector)!=14 or any(not math.isfinite(v) or v<=0 for v in vector):
                raise RuntimeError('factory_holding_parameter_schema_invalid:'+key)
            result[key]=vector
        return result

    def matches(self,values):
        return all(all(abs(a-b)<=1e-6 for a,b in zip(values[key],getattr(self,key))) for key in ('kp','kd'))


def load_holding_profile(root=ROOT):
    import yaml
    values={}
    for name,digest in SOURCES.items():
        path=Path(root)/name
        if path.is_symlink() or not path.resolve().is_relative_to(Path(root).resolve()):
            raise RuntimeError('holding_configuration_path_changed:'+name)
        raw=path.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=digest:
            raise RuntimeError('installed_holding_configuration_changed_review_required:'+name)
        values[name]=yaml.safe_load(raw)
    pairs=[]
    for name in list(SOURCES)[:2]:
        config=values[name]
        if config.get('motor_num')!=31:raise RuntimeError('holding_motor_order_not_verified')
        arrays=[]
        for key in ('kp_pos','kd_pos'):
            vector=config.get(key)
            if not isinstance(vector,list) or len(vector)!=31:raise RuntimeError('holding_full_motor_vector_required')
            arrays.append(tuple(vector[15:29]))
        pairs.append(tuple(arrays))
    if pairs[0]!=pairs[1]:raise RuntimeError('STOP_STANDZERO_arm_gains_disagree')
    return HoldingProfile(*pairs[0],tuple(SOURCES.items()))
