from dataclasses import dataclass
from typing import Optional


@dataclass
class IMUState:
    quat_xyzw: tuple[float, float, float, float] | None = None
    angular_velocity_rad_s: tuple[float, float, float] | None = None
    projected_gravity_b: tuple[float, float, float] | None = None

@dataclass
class RobotPoseState(IMUState):
    projected_gravity_b: tuple[float, float, float] | None = None

    last_quat_t: float = 0.0
    last_gyro_t: float = 0.0