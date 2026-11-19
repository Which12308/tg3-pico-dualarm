"""Our provider contract, NOT fabricated methods on the TianGong public SDK.

The currently available public SDK has no verified dual-arm ownership API.
A separately implemented provider must bind an actual authorized factory
interface and return observed acknowledgements. No JSON/boolean permission
file, guessed FSM state, or reserved-field value implements this contract.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
import math

from .robot import JOINT_IDS


@dataclass(frozen=True)
class FactoryLease:
    owner_id: str
    joint_ids: tuple
    mode: str
    acknowledged_at: float
    expires_at: float
    hand_sides: tuple=()

    def validate(self,now,max_ack_age,required_hands=()):
        if not isinstance(self.owner_id,str) or not 1<=len(self.owner_id)<=256:
            raise ValueError('missing_factory_owner_identity')
        if tuple(self.joint_ids)!=JOINT_IDS or any(type(v) is not int for v in self.joint_ids):
            raise ValueError('factory_lease_does_not_cover_exact_fourteen_arm_joints')
        if not isinstance(self.mode,str) or not self.mode:
            raise ValueError('missing_factory_lease_mode')
        if (not isinstance(self.hand_sides,tuple) or len(set(self.hand_sides))!=len(self.hand_sides)
            or not set(self.hand_sides)<={'left','right'} or not set(required_hands)<=set(self.hand_sides)):
            raise ValueError('factory_hand_ownership_not_acknowledged')
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in (self.acknowledged_at,self.expires_at)):
            raise ValueError('invalid_factory_lease_time')
        if not 0<=now-self.acknowledged_at<=max_ack_age or self.expires_at<now:
            raise ValueError('factory_ownership_ack_missing_or_stale')


@dataclass(frozen=True)
class FactoryStatus:
    lease: FactoryLease | None
    physical_estop: bool | None
    estop_observed_at: float | None
    estop_source: str | None

    def validate_estop(self,now,max_age):
        if self.physical_estop is not False:
            raise ValueError('physical_estop_active_or_unknown')
        stamp=self.estop_observed_at
        if (type(stamp) not in (int,float) or not math.isfinite(stamp) or
            not 0<=now-stamp<=max_age or not isinstance(self.estop_source,str) or not self.estop_source.strip()):
            raise ValueError('physical_estop_feedback_missing_or_stale')


@dataclass(frozen=True)
class FactoryResult:
    acknowledged: bool
    reason: str


class FactoryControlDriver(ABC):
    """Implement against verified manufacturer's native APIs, not mode guesses.

    User motion approval is separate from the runtime ownership acknowledgement.
    read_status must not extend a lease merely because Python is still running;
    acknowledged_at and E-stop time refer to newly received manufacturer data
    in the same monotonic domain used by the adapter.
    """
    @property
    @abstractmethod
    def evidence(self) -> dict:
        """Installed controller identity, version and authoritative interface reference."""

    @abstractmethod
    def request_dual_arm_control(self,hand_sides:tuple=()) -> None:
        """Request both arms and the specified hands without moving them."""

    @abstractmethod
    def read_status(self) -> FactoryStatus:
        """Read actual ownership and independent physical E-stop observations."""

    @abstractmethod
    def hold(self,positions:tuple) -> FactoryResult:
        """Use the manufacturer's verified hold mechanism, with acknowledgement."""

    @abstractmethod
    def stop_and_restore(self,reason:str) -> FactoryResult:
        """Manufacturer-defined stop/recovery; no guessed mode or automatic hand-open."""

    @abstractmethod
    def release_control(self) -> FactoryResult:
        """Release both arm resources through the actual factory API."""
