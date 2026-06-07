from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from qhrr0.factory.robot_factory.base.robot_base import (
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
    ValidationRuleName,
)

from .robot_validation_rules import (
    require_can_interface_supported,
    validate_robot_by_rules,
)


class RobotFactory:
    """Factory for creating validated static Robot data objects."""

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
        required_validation_rules: Iterable[ValidationRuleName] = (),
        metadata: dict[str, Any] | None = None,
        description: str = "",
    ) -> RobotController:
        return RobotController(
            name=name,
            controller_type=controller_type,
            required_validation_rules=tuple(required_validation_rules),
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
        required_validation_rules: Iterable[ValidationRuleName] = (),
        description: str = "",
    ) -> Robot:
        robot = Robot(
            name=name,
            actuators=tuple(actuators),
            imu=imu,
            controller=controller,
            can_interfaces=tuple(can_interfaces),
            required_validation_rules=tuple(required_validation_rules),
            description=description,
        )

        validate_robot_by_rules(robot)
        return robot

    def require_can_interface_supported(self, robot: Robot, interface: str) -> None:
        require_can_interface_supported(robot, interface)
