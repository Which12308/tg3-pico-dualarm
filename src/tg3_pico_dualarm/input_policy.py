"""Explicit session input budgets; transport challenges remain 120 ms."""
from dataclasses import dataclass


@dataclass(frozen=True)
class InputPolicy:
    name: str
    total_age_s: float
    source_age_s: float


NORMAL = InputPolicy('normal', .150, .120)
WIFI_TRIAL = InputPolicy('wifi_trial_200ms', .200, .200)


def policy_for(name='normal'):
    if name == NORMAL.name:
        return NORMAL
    if name == WIFI_TRIAL.name:
        return WIFI_TRIAL
    raise ValueError('unknown_input_freshness_profile')


def from_config(vr):
    policy = policy_for(vr.get('input_profile', 'normal'))
    if (vr['tracking_timeout_s'], vr['source_age_limit_s']) != (policy.total_age_s, policy.source_age_s):
        raise ValueError('changing_input_protocol_freshness_is_not_supported')
    return policy
