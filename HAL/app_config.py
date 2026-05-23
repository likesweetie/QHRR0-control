#!/usr/bin/env python3

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List


@dataclass
class LaunchConfig:
    enable_monitor: bool = True
    monitor_period_s: float = 1.0


@dataclass
class DaemonConfig:
    motor_offsets_deg: List[float] = field(default_factory=lambda: [82.5, -96.0, 159.93])


@dataclass
class TaskControllerConfig:
    home_pos_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.57])
    zero_set_settle_s: float = 2.0
    trajectory_duration_s: float = 6.0
    trajectory_hz: float = 50.0
    hold_hz: float = 5.0
    command_kp: float = 0.0
    command_kd: float = 1.0
    command_v_des: float = 0.0
    command_tau_ff: float = 0.0


@dataclass
class AppConfig:
    launch: LaunchConfig = field(default_factory=LaunchConfig)
    daemon: DaemonConfig = field(default_factory=DaemonConfig)
    task_controller: TaskControllerConfig = field(default_factory=TaskControllerConfig)


def _as_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def load_app_config(path: str = "config.yaml") -> AppConfig:
    cfg = AppConfig()
    p = Path(path)
    if not p.exists():
        return cfg

    try:
        import yaml  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError("PyYAML is required to load config.yaml (pip install pyyaml)") from exc

    with p.open("r", encoding="utf-8") as fp:
        raw = yaml.safe_load(fp) or {}

    top = _as_dict(raw)

    launch = _as_dict(top.get("launch"))
    cfg.launch.enable_monitor = bool(launch.get("enable_monitor", cfg.launch.enable_monitor))
    cfg.launch.monitor_period_s = float(launch.get("monitor_period_s", cfg.launch.monitor_period_s))

    daemon = _as_dict(top.get("daemon"))
    offsets = daemon.get("motor_offsets_deg", cfg.daemon.motor_offsets_deg)
    if isinstance(offsets, list) and all(isinstance(x, (int, float)) for x in offsets):
        cfg.daemon.motor_offsets_deg = [float(x) for x in offsets]

    task = _as_dict(top.get("task_controller"))
    homes = task.get("home_pos_rad", cfg.task_controller.home_pos_rad)
    if isinstance(homes, list) and all(isinstance(x, (int, float)) for x in homes):
        cfg.task_controller.home_pos_rad = [float(x) for x in homes]

    cfg.task_controller.zero_set_settle_s = float(
        task.get("zero_set_settle_s", cfg.task_controller.zero_set_settle_s)
    )
    cfg.task_controller.trajectory_duration_s = float(
        task.get("trajectory_duration_s", cfg.task_controller.trajectory_duration_s)
    )
    cfg.task_controller.trajectory_hz = float(task.get("trajectory_hz", cfg.task_controller.trajectory_hz))
    cfg.task_controller.hold_hz = float(task.get("hold_hz", cfg.task_controller.hold_hz))
    cfg.task_controller.command_kp = float(task.get("command_kp", cfg.task_controller.command_kp))
    cfg.task_controller.command_kd = float(task.get("command_kd", cfg.task_controller.command_kd))
    cfg.task_controller.command_v_des = float(task.get("command_v_des", cfg.task_controller.command_v_des))
    cfg.task_controller.command_tau_ff = float(task.get("command_tau_ff", cfg.task_controller.command_tau_ff))

    return cfg
