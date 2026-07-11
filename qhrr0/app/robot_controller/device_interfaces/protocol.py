from dataclasses import dataclass, field


@dataclass(frozen=True, slots=True)
class Freshness:
    online: bool = False
    stale: bool = True

    last_update_t: float = 0.0
    age_s: float = float("inf")

    rx_count: int = 0
    timeout_count: int = 0
    decode_error_count: int = 0


@dataclass(frozen=True, slots=True)
class ActuatorState:
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0

    enabled: bool = False

    faulted: bool = False
    fault_code: int = 0

    freshness: Freshness = field(default_factory=Freshness)