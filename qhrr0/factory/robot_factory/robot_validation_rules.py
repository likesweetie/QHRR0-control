# robot_factory/validation_policy.py
from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import Any

from robot_factory.base.robot_base import Actuator, IMU, Robot, RobotController


class RobotValidationError(ValueError):
    """Raised when a robot validation policy fails."""


RobotValidator = Callable[[Robot], None]


_VALIDATORS: dict[str, RobotValidator] = {}


def robot_validation(name: str) -> Callable[[RobotValidator], RobotValidator]:
    """Register a robot validation policy.

    Engineers can add a new validation by defining:

        @robot_validation("my_validation")
        def validate_my_rule(robot: Robot) -> None:
            ...

    Then add "my_validation" to:
        - Robot.required_validations, or
        - Robot.controller.required_validations
    """

    if not isinstance(name, str) or not name.strip():
        raise RobotValidationError("validation name must be a non-empty string")

    key = name.strip()

    def decorator(func: RobotValidator) -> RobotValidator:
        if key in _VALIDATORS:
            raise RobotValidationError(f"duplicate robot validation policy: {key}")
        _VALIDATORS[key] = func
        return func

    return decorator


def validate_robot_by_policy(robot: Robot) -> None:
    """Run all validation policies requested by the robot and its controller."""

    _require_instance(robot, Robot, "robot")

    validation_names = collect_required_validations(robot)
    _validate_required_validation_names(validation_names)

    for validation_name in validation_names:
        try:
            validator = _VALIDATORS[validation_name]
        except KeyError as exc:
            available = ", ".join(sorted(_VALIDATORS)) or "<none>"
            raise RobotValidationError(
                f"unknown robot validation policy: {validation_name!r}. "
                f"Available policies: {available}"
            ) from exc

        validator(robot)


def collect_required_validations(robot: Robot) -> tuple[str, ...]:
    """Collect robot-level and controller-level validations.

    Order:
    1. robot.required_validations
    2. robot.controller.required_validations

    Duplicate names are removed while preserving order.
    """

    _require_instance(robot, Robot, "robot")

    names: list[str] = []

    for validation_name in robot.required_validations:
        names.append(validation_name)

    # RobotController를 쓰는 구조라면 controller validation도 자동 포함.
    _require_instance(robot.controller, RobotController, "robot.controller")

    for validation_name in robot.controller.required_validations:
        names.append(validation_name)

    return _deduplicate_preserving_order(names)


def list_robot_validation_policies() -> tuple[str, ...]:
    return tuple(sorted(_VALIDATORS))


@robot_validation("robot_identity")
def validate_robot_identity(robot: Robot) -> None:
    """Validate robot identity fields."""

    _require_non_empty_string(robot.name, "robot.name")

    if not isinstance(robot.description, str):
        raise RobotValidationError("robot.description must be str")


@robot_validation("can_interfaces")
def validate_can_interfaces(robot: Robot) -> None:
    """Validate statically supported CAN interfaces.

    This replaces the old robot_platform.can.allowed_interfaces checks.
    """

    _require_tuple(robot.can_interfaces, "robot.can_interfaces")

    if not robot.can_interfaces:
        raise RobotValidationError("robot.can_interfaces must not be empty")

    for index, interface in enumerate(robot.can_interfaces):
        _require_non_empty_string(
            interface,
            f"robot.can_interfaces[{index}]",
        )

    _require_unique(
        robot.can_interfaces,
        "robot.can_interfaces must not contain duplicates",
    )


@robot_validation("controller")
def validate_controller(robot: Robot) -> None:
    """Validate static controller description.

    This validates only the controller specification, not a runtime controller instance.
    """

    controller = robot.controller

    _require_instance(controller, RobotController, "robot.controller")
    _require_non_empty_string(controller.name, "robot.controller.name")
    _require_non_empty_string(controller.controller_type, "robot.controller.controller_type")

    _require_tuple(
        controller.required_validations,
        "robot.controller.required_validations",
    )

    for index, validation_name in enumerate(controller.required_validations):
        _require_non_empty_string(
            validation_name,
            f"robot.controller.required_validations[{index}]",
        )

    _require_unique(
        controller.required_validations,
        "robot.controller.required_validations must not contain duplicates",
    )

    if controller.metadata is not None and not isinstance(controller.metadata, dict):
        raise RobotValidationError("robot.controller.metadata must be dict or None")

    if not isinstance(controller.description, str):
        raise RobotValidationError("robot.controller.description must be str")


@robot_validation("actuators")
def validate_actuators(robot: Robot) -> None:
    """Validate static actuator definitions.

    This moves these old checks:
    - actuators must not be empty
    - duplicate actuator names
    - duplicate actuator CAN IDs
    - sign must not be zero
    """

    _require_tuple(robot.actuators, "robot.actuators")

    if not robot.actuators:
        raise RobotValidationError("robot.actuators must not be empty")

    actuator_names: list[str] = []
    actuator_can_ids: list[int] = []

    for index, actuator in enumerate(robot.actuators):
        path = f"robot.actuators[{index}]"
        _validate_actuator(actuator, path)

        actuator_names.append(actuator.name)
        actuator_can_ids.append(actuator.can_id)

    _require_unique(
        actuator_names,
        "robot.actuators must not contain duplicate actuator names",
    )

    _require_unique(
        actuator_can_ids,
        "robot.actuators must not contain duplicate actuator CAN IDs",
    )


@robot_validation("imu")
def validate_imu(robot: Robot) -> None:
    """Validate static IMU definition.

    This only validates the simplified IMU object:
        name, driver, can_id

    Driver-specific IMU protocol fields such as request_id, quat_id, gyro_id,
    cmd_get_quat, quat_scale, and gyro_scale should remain in CAN device config
    validation unless those fields are moved into IMU.
    """

    _validate_imu(robot.imu, "robot.imu")


@robot_validation("can_id_conflicts")
def validate_can_id_conflicts(robot: Robot) -> None:
    """Check CAN ID conflicts between actuators and IMU.

    This assumes robot.imu.can_id shares the same CAN arbitration-ID namespace
    as actuator CAN IDs.
    """

    actuator_can_ids = [actuator.can_id for actuator in robot.actuators]
    all_can_ids = actuator_can_ids + [robot.imu.can_id]

    _require_unique(
        all_can_ids,
        "robot CAN IDs must not contain duplicates between actuators and IMU",
    )


@robot_validation("quadruped_12dof_layout")
def validate_quadruped_12dof_layout(robot: Robot) -> None:
    """Validate 12-DoF quadruped semantic actuator layout.

    This is optional. Add this validation only for robots/controllers that assume:
    - 4 hip actuators
    - 4 thigh actuators
    - 4 calf actuators
    """

    group_counts: dict[str, int] = {}

    for actuator in robot.actuators:
        group_counts[actuator.group] = group_counts.get(actuator.group, 0) + 1

    expected = {
        "hip": 4,
        "thigh": 4,
        "calf": 4,
    }

    for group, expected_count in expected.items():
        actual_count = group_counts.get(group, 0)

        if actual_count != expected_count:
            raise RobotValidationError(
                f"robot requires {expected_count} {group} actuators, "
                f"got {actual_count}"
            )


@robot_validation("policy_controller_requires_12_actuators")
def validate_policy_controller_requires_12_actuators(robot: Robot) -> None:
    """Validate controller assumption for 12-dimensional action output."""

    if len(robot.actuators) != 12:
        raise RobotValidationError(
            f"controller {robot.controller.name!r} requires 12 actuators, "
            f"got {len(robot.actuators)}"
        )


@robot_validation("actuator_drivers_known")
def validate_actuator_drivers_known(robot: Robot) -> None:
    """Validate actuator driver names against controller metadata.

    This replaces the old _validate_actuator_drivers style check if you decide
    that allowed actuator drivers belong to the controller/robot definition layer.

    Expected metadata form:
        controller.metadata = {
            "allowed_actuator_drivers": ("spg_mit", "some_other_driver")
        }
    """

    metadata = robot.controller.metadata or {}
    allowed = metadata.get("allowed_actuator_drivers")

    if allowed is None:
        raise RobotValidationError(
            "robot.controller.metadata.allowed_actuator_drivers is required "
            "for validation 'actuator_drivers_known'"
        )

    if not isinstance(allowed, tuple):
        raise RobotValidationError(
            "robot.controller.metadata.allowed_actuator_drivers must be tuple"
        )

    for index, driver_name in enumerate(allowed):
        _require_non_empty_string(
            driver_name,
            f"robot.controller.metadata.allowed_actuator_drivers[{index}]",
        )

    allowed_set = set(allowed)

    for actuator in robot.actuators:
        if actuator.driver not in allowed_set:
            raise RobotValidationError(
                f"actuator {actuator.name!r} references unknown driver: "
                f"{actuator.driver!r}"
            )


def require_can_interface_supported(robot: Robot, interface: str) -> None:
    """Runtime-side check for selected CAN interface.

    This can be called by config validation or runtime safety validation.
    It belongs here because the allowed interface list is now part of Robot.
    """

    _require_instance(robot, Robot, "robot")
    _require_non_empty_string(interface, "can.interface")

    if interface not in robot.can_interfaces:
        allowed = ", ".join(robot.can_interfaces) or "<none>"
        raise RobotValidationError(
            f"CAN interface {interface!r} is not supported by robot {robot.name!r}. "
            f"Allowed interfaces: {allowed}"
        )


def _validate_required_validation_names(validation_names: tuple[str, ...]) -> None:
    if not validation_names:
        raise RobotValidationError("robot validation policy list must not be empty")

    for index, validation_name in enumerate(validation_names):
        _require_non_empty_string(validation_name, f"required_validations[{index}]")


def _validate_actuator(actuator: object, path: str) -> None:
    _require_instance(actuator, Actuator, path)

    _require_non_empty_string(actuator.name, f"{path}.name")
    _require_non_empty_string(actuator.driver, f"{path}.driver")

    if not isinstance(actuator.group, str):
        raise RobotValidationError(f"{path}.group must be str")

    _require_int(actuator.can_id, f"{path}.can_id")
    if actuator.can_id <= 0:
        raise RobotValidationError(f"{path}.can_id must be > 0")

    _require_number(actuator.sign, f"{path}.sign")
    if actuator.sign == 0.0:
        raise RobotValidationError(f"{path}.sign must not be 0")

    _require_number(actuator.offset_rad, f"{path}.offset_rad")


def _validate_imu(imu: object, path: str) -> None:
    _require_instance(imu, IMU, path)

    _require_non_empty_string(imu.name, f"{path}.name")
    _require_non_empty_string(imu.driver, f"{path}.driver")

    _require_int(imu.can_id, f"{path}.can_id")
    if imu.can_id <= 0:
        raise RobotValidationError(f"{path}.can_id must be > 0")


def _require_instance(value: object, expected_type: type, path: str) -> None:
    if not isinstance(value, expected_type):
        raise RobotValidationError(
            f"{path} must be {expected_type.__name__}, got {type(value).__name__}"
        )


def _require_tuple(value: object, path: str) -> None:
    if not isinstance(value, tuple):
        raise RobotValidationError(f"{path} must be tuple")


def _require_non_empty_string(value: object, path: str) -> None:
    if not isinstance(value, str):
        raise RobotValidationError(
            f"{path} must be str, got {type(value).__name__}"
        )

    if not value.strip():
        raise RobotValidationError(f"{path} must not be empty")


def _require_int(value: object, path: str) -> None:
    # bool is a subclass of int, so reject it explicitly.
    if isinstance(value, bool) or not isinstance(value, int):
        raise RobotValidationError(
            f"{path} must be int, got {type(value).__name__}"
        )


def _require_number(value: object, path: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise RobotValidationError(
            f"{path} must be int or float, got {type(value).__name__}"
        )


def _require_unique(values: Iterable[object], message: str) -> None:
    items = list(values)

    if len(set(items)) != len(items):
        raise RobotValidationError(message)


def _deduplicate_preserving_order(values: Iterable[str]) -> tuple[str, ...]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return tuple(result)