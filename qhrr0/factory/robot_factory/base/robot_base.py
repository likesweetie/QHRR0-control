# robot_factory/base/robot_base.py
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
ValidationName: TypeAlias = str


@dataclass(frozen=True, slots=True, kw_only=True)
class Actuator:
    """Static actuator data.

    This class is intentionally a pure data container.
    Do not add validation, type conversion, derived properties, or lookup logic here.
    """

    name: ActuatorName
    driver: DriverName
    can_id: int

    sign: float
    offset_rad: float

    # Free-form semantic group.
    # Examples: "hip", "thigh", "calf", "wheel", "gripper".
    group: str = ""


@dataclass(frozen=True, slots=True, kw_only=True)
class IMU:
    """Static IMU data.

    This class is intentionally a pure data container.
    Do not add validation, type conversion, derived properties, or lookup logic here.
    """

    name: ImuName
    driver: DriverName
    can_id: int


@dataclass(frozen=True, slots=True, kw_only=True)
class RobotController:
    """Static robot controller data.

    This is not a runtime controller instance.
    It only describes which controller profile the robot uses and which
    validations are required before the Robot object is accepted by the factory.
    """

    name: ControllerName

    controller_type: ControllerType

    required_validations: tuple[ValidationName, ...] = ()

    # Optional free-form metadata.
    # This should stay as static data only.
    # Runtime objects such as sockets, shm handles, process handles, ONNX sessions,
    # or controller class instances must not be stored here.
    metadata: dict[str, Any] | None = None

    description: str = ""


@dataclass(frozen=True, slots=True, kw_only=True)
class Robot:
    """Static robot data.

    This class is intentionally a pure data container.

    It should describe what the robot is:
    - robot identity
    - supported CAN interfaces
    - actuator information
    - IMU information
    - controller definition

    It should not contain:
    - validation logic
    - runtime state
    - CAN sockets
    - shared-memory handles
    - policy runners
    - actual controller instances
    - process handles
    """

    name: RobotName

    actuators: tuple[Actuator, ...]
    imu: IMU

    controller: RobotController

    allowed_can_interfaces: tuple[CanInterfaceName, ...] = ()

    # Robot-level validation requirements.
    # Controller-specific validations should be placed in controller.required_validations.
    required_validations: tuple[ValidationName, ...] = ()

    description: str = ""