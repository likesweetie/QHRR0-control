from __future__ import annotations

from dataclasses import dataclass
from typing import Any, TypeAlias


RobotName: TypeAlias = str
RobotRevision: TypeAlias = str

ActuatorName: TypeAlias = str
DriverName: TypeAlias = str
CanInterfaceName: TypeAlias = str
ImuName: TypeAlias = str

ControllerName: TypeAlias = str
ControllerType: TypeAlias = str
ValidationRuleName: TypeAlias = str


@dataclass(frozen=True, slots=True, kw_only=True)
class Actuator:
    name: ActuatorName
    driver: DriverName
    can_id: int
    sign: float
    offset_rad: float
    group: str = ""


@dataclass(frozen=True, slots=True, kw_only=True)
class IMU:
    name: ImuName
    driver: DriverName
    can_id: int


@dataclass(frozen=True, slots=True, kw_only=True)
class RobotController:
    name: ControllerName
    controller_type: ControllerType
    required_validation_rules: tuple[ValidationRuleName, ...] = ()
    metadata: dict[str, Any] | None = None
    description: str = ""


@dataclass(frozen=True, slots=True, kw_only=True)
class Robot:
    name: RobotName
    actuators: tuple[Actuator, ...]
    imu: IMU
    controller: RobotController
    can_interfaces: tuple[CanInterfaceName, ...] = ()
    required_validation_rules: tuple[ValidationRuleName, ...] = ()
    description: str = ""
