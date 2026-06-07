from __future__ import annotations

from collections.abc import Callable, Iterable

from qhrr0.factory.robot_factory.base.robot_base import (
    Actuator,
    IMU,
    Robot,
    RobotController,
)


class RobotValidationError(ValueError):
    """Raised when a static robot validation rule fails."""


RobotValidationRule = Callable[[Robot], None]


_ROBOT_VALIDATION_RULES: dict[str, RobotValidationRule] = {}


def robot_validation_rule(
    name: str,
) -> Callable[[RobotValidationRule], RobotValidationRule]:
    if not isinstance(name, str) or not name.strip():
        raise RobotValidationError("robot validation rule name must be a non-empty string")

    key = name.strip()

    def decorator(func: RobotValidationRule) -> RobotValidationRule:
        if key in _ROBOT_VALIDATION_RULES:
            raise RobotValidationError(f"duplicate robot validation rule: {key}")
        _ROBOT_VALIDATION_RULES[key] = func
        return func

    return decorator


def validate_robot_by_rules(robot: Robot) -> None:
    _require_instance(robot, Robot, "robot")

    rule_names = collect_required_validation_rules(robot)
    _validate_required_rule_names(rule_names)

    for rule_name in rule_names:
        try:
            rule = _ROBOT_VALIDATION_RULES[rule_name]
        except KeyError as exc:
            available = ", ".join(sorted(_ROBOT_VALIDATION_RULES)) or "<none>"
            raise RobotValidationError(
                f"unknown robot validation rule: {rule_name!r}. "
                f"Available rules: {available}"
            ) from exc

        rule(robot)


def collect_required_validation_rules(robot: Robot) -> tuple[str, ...]:
    _require_instance(robot, Robot, "robot")
    _require_instance(robot.controller, RobotController, "robot.controller")

    names: list[str] = []
    names.extend(robot.required_validation_rules)
    names.extend(robot.controller.required_validation_rules)
    return _deduplicate_preserving_order(names)


def list_robot_validation_rules() -> tuple[str, ...]:
    return tuple(sorted(_ROBOT_VALIDATION_RULES))


@robot_validation_rule("robot_identity")
def validate_robot_identity(robot: Robot) -> None:
    _require_non_empty_string(robot.name, "robot.name")

    if not isinstance(robot.description, str):
        raise RobotValidationError("robot.description must be str")


@robot_validation_rule("can_interfaces")
def validate_can_interfaces(robot: Robot) -> None:
    _require_tuple(robot.can_interfaces, "robot.can_interfaces")

    if not robot.can_interfaces:
        raise RobotValidationError("robot.can_interfaces must not be empty")

    for index, interface in enumerate(robot.can_interfaces):
        _require_non_empty_string(interface, f"robot.can_interfaces[{index}]")

    _require_unique(
        robot.can_interfaces,
        "robot.can_interfaces must not contain duplicates",
    )


@robot_validation_rule("controller")
def validate_controller(robot: Robot) -> None:
    controller = robot.controller

    _require_instance(controller, RobotController, "robot.controller")
    _require_non_empty_string(controller.name, "robot.controller.name")
    _require_non_empty_string(controller.controller_type, "robot.controller.controller_type")

    _require_tuple(
        controller.required_validation_rules,
        "robot.controller.required_validation_rules",
    )

    for index, rule_name in enumerate(controller.required_validation_rules):
        _require_non_empty_string(
            rule_name,
            f"robot.controller.required_validation_rules[{index}]",
        )

    _require_unique(
        controller.required_validation_rules,
        "robot.controller.required_validation_rules must not contain duplicates",
    )

    if controller.metadata is not None and not isinstance(controller.metadata, dict):
        raise RobotValidationError("robot.controller.metadata must be dict or None")

    if not isinstance(controller.description, str):
        raise RobotValidationError("robot.controller.description must be str")


@robot_validation_rule("actuators")
def validate_actuators(robot: Robot) -> None:
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


@robot_validation_rule("imu")
def validate_imu(robot: Robot) -> None:
    _validate_imu(robot.imu, "robot.imu")


@robot_validation_rule("can_id_conflicts")
def validate_can_id_conflicts(robot: Robot) -> None:
    can_ids = [actuator.can_id for actuator in robot.actuators] + [robot.imu.can_id]
    _require_unique(
        can_ids,
        "robot CAN IDs must not contain duplicates between actuators and IMU",
    )


@robot_validation_rule("actuator_drivers_known")
def validate_actuator_drivers_known(robot: Robot) -> None:
    metadata = robot.controller.metadata or {}
    allowed = metadata.get("allowed_actuator_drivers")

    if allowed is None:
        raise RobotValidationError(
            "robot.controller.metadata.allowed_actuator_drivers is required "
            "for validation rule 'actuator_drivers_known'"
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


@robot_validation_rule("single_leg_3dof_layout")
def validate_single_leg_3dof_layout(robot: Robot) -> None:
    actual = {actuator.group for actuator in robot.actuators}
    expected = {"hip_roll", "hip_pitch", "knee_pitch"}

    if actual != expected:
        raise RobotValidationError(
            "QHRR0 single-leg layout requires actuator groups "
            f"{sorted(expected)}, got {sorted(actual)}"
        )


@robot_validation_rule("policy_controller_requires_3_actuators")
def validate_policy_controller_requires_3_actuators(robot: Robot) -> None:
    if len(robot.actuators) != 3:
        raise RobotValidationError(
            f"controller {robot.controller.name!r} requires 3 actuators, "
            f"got {len(robot.actuators)}"
        )


def require_can_interface_supported(robot: Robot, interface: str) -> None:
    _require_instance(robot, Robot, "robot")
    _require_non_empty_string(interface, "can.interface")

    if interface not in robot.can_interfaces:
        allowed = ", ".join(robot.can_interfaces) or "<none>"
        raise RobotValidationError(
            f"CAN interface {interface!r} is not supported by robot {robot.name!r}. "
            f"Allowed interfaces: {allowed}"
        )


def _validate_required_rule_names(rule_names: tuple[str, ...]) -> None:
    if not rule_names:
        raise RobotValidationError("robot validation rule list must not be empty")

    for index, rule_name in enumerate(rule_names):
        _require_non_empty_string(rule_name, f"required_validation_rules[{index}]")


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
