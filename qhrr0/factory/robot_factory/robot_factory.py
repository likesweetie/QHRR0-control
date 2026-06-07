# robot_factory/robot_factory.py
from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from robot_factory.base.robot_base import (
    Actuator,
    ActuatorName,
    CanInterfaceName,
    ControllerName,
    ControllerType,
    DriverName,
    IMU,
    ImuName,
    Robot,
    RobotController,
    RobotName,
    ValidationName,
)

from .validation_policy import (
    require_can_interface_supported,
    validate_robot_by_policy,
)


class RobotFactory:
    """Factory for creating validated Robot data objects.

    Robot, Actuator, IMU, and RobotController are pure data containers.

    This factory owns:
    - object construction
    - tuple normalization
    - validation-policy execution

    validation_policy.py owns:
    - actual validation rules
    - validation registry
    """

    def create_actuator(
        self,
        *,
        name: ActuatorName,
        driver: DriverName,
        can_id: int,
        sign: float,
        offset_rad: float,
        group: str = "",
    ) -> Actuator:
        return Actuator(
            name=name,
            driver=driver,
            can_id=can_id,
            sign=sign,
            offset_rad=offset_rad,
            group=group,
        )

    def create_imu(
        self,
        *,
        name: ImuName,
        driver: DriverName,
        can_id: int,
    ) -> IMU:
        return IMU(
            name=name,
            driver=driver,
            can_id=can_id,
        )

    def create_controller(
        self,
        *,
        name: ControllerName,
        controller_type: ControllerType,
        required_validations: Iterable[ValidationName] = (),
        metadata: dict[str, Any] | None = None,
        description: str = "",
    ) -> RobotController:
        return RobotController(
            name=name,
            controller_type=controller_type,
            required_validations=tuple(required_validations),
            metadata=metadata,
            description=description,
        )

    def create_robot(
        self,
        *,
        name: RobotName,
        actuators: Iterable[Actuator],
        imu: IMU,
        controller: RobotController,
        can_interfaces: Iterable[CanInterfaceName] = (),
        required_validations: Iterable[ValidationName] = (),
        description: str = "",
    ) -> Robot:
        robot = Robot(
            name=name,
            actuators=tuple(actuators),
            imu=imu,
            controller=controller,
            can_interfaces=tuple(can_interfaces),
            required_validations=tuple(required_validations),
            description=description,
        )

        validate_robot_by_policy(robot)
        return robot

    def require_can_interface_supported(self, robot: Robot, interface: str) -> None:
        require_can_interface_supported(robot, interface)