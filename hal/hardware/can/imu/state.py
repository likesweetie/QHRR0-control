from dataclasses import dataclass
from typing import Optional


@dataclass
class IMUState:
    quat_xyzw: tuple[float, float, float, float] | None = None
    angular_velocity_rad_s: tuple[float, float, float] | None = None
