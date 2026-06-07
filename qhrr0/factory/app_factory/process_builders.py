from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Mapping

from qhrr0.factory.robot_factory.base.robot_base import Robot
from qhrr0.qhrr0_spec import QHRR0

from .app_validation_rules import AppConfigError, value
from .loaders import config_path, policy_path
from .schema import ProcessLaunchSpec


DEFAULT_DASHBOARD_RUNTIME_CONFIG = Path("/tmp/qhrr0/dashboard_runtime.json")

ProcessBuilder = Callable[
    [Mapping[str, Any], Mapping[str, Any], Mapping[str, Mapping[str, Path]], Path],
    ProcessLaunchSpec,
]


def build_process_launch_specs(
    config: Mapping[str, Any],
    config_paths: Mapping[str, Mapping[str, Path]],
    *,
    dashboard_runtime_config_path: str | Path = DEFAULT_DASHBOARD_RUNTIME_CONFIG,
    robot: Robot = QHRR0,
) -> tuple[ProcessLaunchSpec, ...]:
    _require_qhrr0_runtime(robot)
    runtime_path = Path(dashboard_runtime_config_path)
    specs = []
    for process in value(config, "processes.orchestration"):
        name = str(process["name"])
        try:
            builder = PROCESS_BUILDERS[name]
        except KeyError as exc:
            raise AppConfigError(f"unsupported process for explicit qhrr0 runtime wiring: {name}") from exc
        specs.append(builder(process, config, config_paths, runtime_path))
    return tuple(specs)


def build_can_daemon_process(
    orchestration: Mapping[str, Any],
    config: Mapping[str, Any],
    _config_paths: Mapping[str, Mapping[str, Path]],
    _dashboard_runtime_config_path: Path,
) -> ProcessLaunchSpec:
    subprocess_config = value(config, "processes.subprocesses.can_daemon")
    command = [
        "python3",
        "-m",
        "qhrr0.app.robot_controller.subprocesses.can_daemon.main",
        "--can-interface",
        str(value(config, "can.interface")),
        "--ipc-socket-path",
        str(value(config, "can.daemon.ipc_socket_path")),
        "--rx-timeout-s",
        str(value(config, "can.daemon.rx_timeout_s")),
        "--tx-timeout-s",
        str(value(config, "can.daemon.tx_timeout_s")),
        "--join-timeout-s",
        str(value(config, "can.daemon.join_timeout_s")),
        "--max-tx-queue-size",
        str(value(config, "can.daemon.max_tx_queue_size")),
    ]
    if bool(value(config, "can.daemon.send_block")):
        command.append("--send-block")
    if value(config, "can.daemon.send_timeout_s") is not None:
        command.extend(["--send-timeout-s", str(value(config, "can.daemon.send_timeout_s"))])
    if bool(subprocess_config["replace_existing_socket"]):
        command.append("--replace-existing-socket")
    return _with_orchestration(orchestration, command)


def build_aux_reader_process(
    orchestration: Mapping[str, Any],
    config: Mapping[str, Any],
    _config_paths: Mapping[str, Mapping[str, Path]],
    _dashboard_runtime_config_path: Path,
) -> ProcessLaunchSpec:
    subprocess_config = value(config, "processes.subprocesses.aux_reader")
    return _with_orchestration(
        orchestration,
        [
            "python3",
            "-m",
            "qhrr0.app.robot_controller.subprocesses.aux_reader.main",
            "--aux-command-shm-name",
            str(value(config, "shm.aux_command.name")),
            "--joystick-dev",
            str(subprocess_config["joystick_dev"]),
        ],
    )


def build_task_controller_process(
    orchestration: Mapping[str, Any],
    config: Mapping[str, Any],
    config_paths: Mapping[str, Mapping[str, Path]],
    _dashboard_runtime_config_path: Path,
) -> ProcessLaunchSpec:
    subprocess_config = value(config, "processes.subprocesses.task_controller")
    command = [
        "python3",
        "-m",
        "qhrr0.app.robot_controller.subprocesses.task_controller.main",
        "--policy-runner-config",
        str(config_path(config_paths, "policy_runner")),
        "--policy-list",
        str(policy_path(config_paths, "policy_list")),
        "--pd-config",
        str(policy_path(config_paths, "pd_config")),
        "--robot-name",
        str(value(config, "robot.name")),
        "--can-ids",
        ",".join(str(can_id) for can_id in value(config, "can.motors.can_ids")),
        "--control-state-shm-name",
        str(value(config, "shm.control_state.name")),
        "--aux-command-shm-name",
        str(value(config, "shm.aux_command.name")),
        "--mit-command-shm-name",
        str(value(config, "shm.mit_command.name")),
        "--project-root",
        str(subprocess_config["project_root"]),
        "--control-hz",
        str(subprocess_config["control_hz"]),
    ]
    rate_log_interval_s = subprocess_config["rate_log_interval_s"]
    if rate_log_interval_s is not None:
        command.extend(["--rate-log-interval-s", str(rate_log_interval_s)])
    return _with_orchestration(orchestration, command)


def build_dashboard_process(
    orchestration: Mapping[str, Any],
    _config: Mapping[str, Any],
    _config_paths: Mapping[str, Mapping[str, Path]],
    dashboard_runtime_config_path: Path,
) -> ProcessLaunchSpec:
    return _with_orchestration(
        orchestration,
        [
            "python3",
            "-m",
            "qhrr0.app.robot_controller.subprocesses.dashboard.main",
            "--runtime-config",
            str(dashboard_runtime_config_path),
        ],
    )


PROCESS_BUILDERS: dict[str, ProcessBuilder] = {
    "can_daemon": build_can_daemon_process,
    "aux_reader": build_aux_reader_process,
    "task_controller": build_task_controller_process,
    "dashboard": build_dashboard_process,
}


def _with_orchestration(
    orchestration: Mapping[str, Any],
    command: list[str],
) -> ProcessLaunchSpec:
    return ProcessLaunchSpec(
        name=str(orchestration["name"]),
        command=tuple(str(part) for part in command),
        start_order=int(orchestration["start_order"]),
        stop_order=int(orchestration["stop_order"]),
        new_terminal=bool(orchestration["new_terminal"]),
        terminal_command=tuple(str(part) for part in orchestration["terminal_command"]),
        working_dir=str(orchestration["working_dir"]),
        env_vars={str(key): str(value) for key, value in orchestration["env_vars"].items()},
    )


def _require_qhrr0_runtime(robot: Robot) -> None:
    if str(robot.name) != str(QHRR0.name):
        raise AppConfigError("AppFactory runtime wiring is qhrr0-specific")
