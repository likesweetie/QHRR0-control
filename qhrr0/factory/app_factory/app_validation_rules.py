# factory/app_factory/app_validation_rules.py
from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Any

from factory.robot_factory.base.robot_base import Robot
from factory.robot_factory.robot_validation_rules import require_can_interface_supported


class AppValidationError(ValueError):
    """Raised when app/config validation fails during factory stage."""


@dataclass(frozen=True, slots=True, kw_only=True)
class AppValidationContext:
    """Context for app validation rules.

    Config objects are allowed here because this context exists only during
    factory/build stage. Runtime objects must not keep this context.
    """

    config: Any
    robot: Robot
    options: Any | None = None


AppValidationRule = Callable[[AppValidationContext], None]


_APP_VALIDATION_RULES: dict[str, AppValidationRule] = {}


def app_validation_rule(name: str) -> Callable[[AppValidationRule], AppValidationRule]:
    if not isinstance(name, str) or not name.strip():
        raise AppValidationError("app validation rule name must be a non-empty string")

    key = name.strip()

    def decorator(func: AppValidationRule) -> AppValidationRule:
        if key in _APP_VALIDATION_RULES:
            raise AppValidationError(f"duplicate app validation rule: {key}")
        _APP_VALIDATION_RULES[key] = func
        return func

    return decorator


def validate_app_by_rules(
    ctx: AppValidationContext,
    required_validation_rules: Iterable[str],
) -> None:
    rule_names = _deduplicate_preserving_order(required_validation_rules)

    if not rule_names:
        raise AppValidationError("app validation rule list must not be empty")

    for rule_name in rule_names:
        try:
            rule = _APP_VALIDATION_RULES[rule_name]
        except KeyError as exc:
            available = ", ".join(sorted(_APP_VALIDATION_RULES)) or "<none>"
            raise AppValidationError(
                f"unknown app validation rule: {rule_name!r}. "
                f"Available rules: {available}"
            ) from exc

        rule(ctx)


def list_app_validation_rules() -> tuple[str, ...]:
    return tuple(sorted(_APP_VALIDATION_RULES))


@app_validation_rule("runtime_mode")
def validate_runtime_mode(ctx: AppValidationContext) -> None:
    if ctx.config.runtime.mode not in ("simulation", "hardware"):
        raise AppValidationError("runtime.mode must be 'simulation' or 'hardware'")


@app_validation_rule("can_interface_supported_by_robot")
def validate_can_interface_supported_by_robot(ctx: AppValidationContext) -> None:
    require_can_interface_supported(ctx.robot, str(ctx.config.can.interface))


@app_validation_rule("safety_config")
def validate_safety_config(ctx: AppValidationContext) -> None:
    safety = ctx.config.safety

    if safety.velocity_damping_kd < 0.0:
        raise AppValidationError("safety.velocity_damping_kd must be >= 0")

    if safety.damping_timeout_s <= 0.0:
        raise AppValidationError("safety.damping_timeout_s must be > 0")

    if safety.command_loss_action not in ("damping", "disable", "fault"):
        raise AppValidationError(
            "safety.command_loss_action must be damping, disable, or fault"
        )

    if safety.feedback_stale_action not in ("damping", "disable", "fault"):
        raise AppValidationError(
            "safety.feedback_stale_action must be damping, disable, or fault"
        )


@app_validation_rule("robot_controller_config")
def validate_robot_controller_config(ctx: AppValidationContext) -> None:
    robot_controller = ctx.config.robot_controller

    if robot_controller.control_hz <= 0.0:
        raise AppValidationError("robot_controller.control_hz must be > 0")

    if robot_controller.shutdown_timeout_s < 0.0:
        raise AppValidationError("robot_controller.shutdown_timeout_s must be >= 0")


@app_validation_rule("state_machine_config")
def validate_state_machine_config(ctx: AppValidationContext) -> None:
    state_machine = ctx.config.state_machine

    if state_machine.enable_duration_s < 0.0:
        raise AppValidationError("state_machine.enable_duration_s must be >= 0")


@app_validation_rule("safety_vs_protocol_range")
def validate_safety_vs_protocol_range(ctx: AppValidationContext) -> None:
    if ctx.config.safety.velocity_damping_kd > ctx.config.can.mit_protocol_range.kd:
        raise AppValidationError(
            "safety.velocity_damping_kd must be <= can.mit_protocol_range.kd"
        )


@app_validation_rule("can_config")
def validate_can_config(ctx: AppValidationContext) -> None:
    can = ctx.config.can

    if not can.interface:
        raise AppValidationError("can.interface must not be empty")

    if can.bitrate <= 0:
        raise AppValidationError("can.bitrate must be > 0")

    if can.command_timeout_s <= 0.0:
        raise AppValidationError("can.command_timeout_s must be > 0")

    if can.bringup_delay_s < 0.0:
        raise AppValidationError("can.bringup_delay_s must be >= 0")

    if can.daemon.rx_timeout_s < 0.0:
        raise AppValidationError("can.daemon.rx_timeout_s must be >= 0")

    if can.daemon.tx_timeout_s < 0.0:
        raise AppValidationError("can.daemon.tx_timeout_s must be >= 0")

    if can.daemon.join_timeout_s < 0.0:
        raise AppValidationError("can.daemon.join_timeout_s must be >= 0")

    if can.daemon.max_tx_queue_size <= 0:
        raise AppValidationError("can.daemon.max_tx_queue_size must be > 0")

    if can.daemon.send_timeout_s is not None and can.daemon.send_timeout_s < 0.0:
        raise AppValidationError(
            "can.daemon.send_timeout_s must be null or >= 0"
        )

    if not can.daemon.ipc_socket_path:
        raise AppValidationError("can.daemon.ipc_socket_path must not be empty")

    if can.daemon.connect_timeout_s <= 0.0:
        raise AppValidationError("can.daemon.connect_timeout_s must be > 0")

    if can.imu.startup_request_count < 0:
        raise AppValidationError("can.imu.startup_request_count must be >= 0")

    if can.imu.startup_request_delay_s < 0.0:
        raise AppValidationError("can.imu.startup_request_delay_s must be >= 0")


@app_validation_rule("mit_protocol_range")
def validate_mit_protocol_range(ctx: AppValidationContext) -> None:
    protocol = ctx.config.can.mit_protocol_range

    if protocol.position_rad <= 0.0:
        raise AppValidationError("can.mit_protocol_range.position_rad must be > 0")

    if protocol.velocity_rad_s <= 0.0:
        raise AppValidationError("can.mit_protocol_range.velocity_rad_s must be > 0")

    if protocol.kp < 0.0:
        raise AppValidationError("can.mit_protocol_range.kp must be >= 0")

    if protocol.kd < 0.5:
        raise AppValidationError(
            "can.mit_protocol_range.kd must be >= 0.5 for shutdown damping"
        )

    if protocol.torque_ff_nm <= 0.0:
        raise AppValidationError("can.mit_protocol_range.torque_ff_nm must be > 0")

    if protocol.feedback_position_rad <= 0.0:
        raise AppValidationError(
            "can.mit_protocol_range.feedback_position_rad must be > 0"
        )


@app_validation_rule("processes")
def validate_processes(ctx: AppValidationContext) -> None:
    processes = ctx.config.processes

    names = [process.name for process in processes]

    if len(set(names)) != len(names):
        raise AppValidationError("processes must not contain duplicate names")

    if "can_daemon" not in set(names):
        raise AppValidationError("processes must include a 'can_daemon' subprocess")

    for process in processes:
        if not process.name:
            raise AppValidationError("process name must not be empty")

        if not process.command:
            raise AppValidationError(
                f"process {process.name} command must not be empty"
            )

        if not process.working_dir:
            raise AppValidationError(
                f"process {process.name} working_dir must not be empty"
            )

        if process.new_terminal and not process.terminal_command:
            raise AppValidationError(
                f"process {process.name} terminal_command must not be empty "
                "when new_terminal is true"
            )


@app_validation_rule("shm")
def validate_shm(ctx: AppValidationContext) -> None:
    shm = ctx.config.shm

    if shm.mit_command.target_count <= 0:
        raise AppValidationError("shm.mit_command.target_count must be > 0")

    if shm.mit_command.target_count != len(ctx.robot.actuators):
        raise AppValidationError(
            "shm.mit_command.target_count must match robot actuator count"
        )

    _validate_shm_segment_size(
        shm.control_state.size_bytes,
        "shm.control_state.size_bytes",
    )
    _validate_positive_float(
        shm.control_state.publish_hz,
        "shm.control_state.publish_hz",
    )

    _validate_shm_segment_size(
        shm.aux_command.size_bytes,
        "shm.aux_command.size_bytes",
    )
    _validate_positive_float(
        shm.aux_command.publish_hz,
        "shm.aux_command.publish_hz",
    )

    _validate_shm_segment_size(
        shm.operator_command.size_bytes,
        "shm.operator_command.size_bytes",
    )

    _validate_shm_segment_size(
        shm.dashboard_state.size_bytes,
        "shm.dashboard_state.size_bytes",
    )
    _validate_positive_float(
        shm.dashboard_state.publish_hz,
        "shm.dashboard_state.publish_hz",
    )

    names = {
        shm.mit_command.name,
        shm.aux_command.name,
        shm.operator_command.name,
        shm.control_state.name,
        shm.dashboard_state.name,
    }

    if len(names) != 5:
        raise AppValidationError("shm segment names must be unique")


@app_validation_rule("hardware_safety")
def validate_hardware_safety(ctx: AppValidationContext) -> None:
    config = ctx.config
    options = ctx.options

    if options is None:
        raise AppValidationError(
            "options is required for validation rule 'hardware_safety'"
        )

    interface = str(config.can.interface)
    is_virtual_can = interface.startswith("vcan")
    is_real_can = interface.startswith("can")

    if config.runtime.mode == "simulation":
        if is_real_can:
            raise AppValidationError("simulation mode rejects real CAN interface")
        return

    if config.runtime.mode != "hardware":
        raise AppValidationError("runtime.mode must be 'simulation' or 'hardware'")

    if not options.hardware_requested:
        raise AppValidationError("hardware mode requires --hardware")

    if not options.motor_enable_confirmed:
        raise AppValidationError(
            "hardware mode requires --i-understand-this-can-enable-motors"
        )

    if is_virtual_can:
        raise AppValidationError("hardware mode rejects virtual CAN interface")

    if not is_real_can:
        raise AppValidationError("hardware mode requires a real CAN interface")

    if not config.hardware.allow_real_can:
        raise AppValidationError("hardware mode requires hardware.allow_real_can")

    if not options.estop_ok:
        raise AppValidationError("hardware mode requires --estop-ok")


def _validate_shm_segment_size(size_bytes: int, path: str) -> None:
    if size_bytes < 4096:
        raise AppValidationError(f"{path} must be >= 4096")


def _validate_positive_float(value: float, path: str) -> None:
    if value <= 0.0:
        raise AppValidationError(f"{path} must be > 0")


def _deduplicate_preserving_order(values: Iterable[str]) -> tuple[str, ...]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return tuple(result)