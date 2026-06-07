from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from qhrr0.factory.robot_factory.base.robot_base import Robot
from qhrr0.qhrr0_spec import QHRR0

from .app_validation_rules import (
    AppConfigError,
    require_robot_matches_config,
    validate_app_config,
    validate_runtime_safety,
)
from .controller_runtime import build_mit_config, build_robot_controller_runtime
from .dashboard_runtime import (
    build_dashboard_runtime,
    load_dashboard_runtime_config,
    write_dashboard_runtime_config,
)
from .loaders import (
    DEFAULT_CONFIG_PATHS,
    DEFAULT_CONTROLLER_CONFIG,
    config_path,
    load_can_device_config,
    load_config_paths,
    load_processes_config,
    load_robot_controller_config,
    load_yaml_mapping,
    policy_path,
)
from .process_builders import (
    DEFAULT_DASHBOARD_RUNTIME_CONFIG,
    build_process_launch_specs,
)
from .schema import AppRuntimeSpec


class AppFactory:
    def create_robot_controller_runtime_spec(
        self,
        *,
        controller_config_path: str | Path | None = None,
        config_paths_path: str | Path = DEFAULT_CONFIG_PATHS,
        robot: Robot = QHRR0,
        hardware_requested: bool = False,
        motor_enable_confirmed: bool = False,
        estop_ok: bool = False,
    ) -> AppRuntimeSpec:
        config_paths = load_config_paths(config_paths_path)
        app_config = load_robot_controller_config(
            controller_config_path,
            config_paths=config_paths,
            robot=robot,
        )
        options = {
            "hardware_requested": bool(hardware_requested),
            "motor_enable_confirmed": bool(motor_enable_confirmed),
            "estop_ok": bool(estop_ok),
        }
        self.validate_app(robot=robot, config=app_config, options=options)
        controller_runtime = build_robot_controller_runtime(app_config, robot=robot)
        processes = build_process_launch_specs(
            app_config,
            config_paths,
            dashboard_runtime_config_path=DEFAULT_DASHBOARD_RUNTIME_CONFIG,
            robot=robot,
        )
        dashboard_runtime = build_dashboard_runtime(
            app_config,
            config_paths,
            processes=processes,
            runtime_config_path=DEFAULT_DASHBOARD_RUNTIME_CONFIG,
            robot=robot,
        )
        write_dashboard_runtime_config(dashboard_runtime)
        return AppRuntimeSpec(
            controller=controller_runtime,
            processes=processes,
            dashboard=dashboard_runtime,
        )

    def validate_app(
        self,
        *,
        robot: Robot = QHRR0,
        config: Mapping[str, Any],
        options: Mapping[str, Any] | None = None,
    ) -> None:
        validate_app_config(config)
        validate_runtime_safety(config, {} if options is None else options)
        require_robot_matches_config(robot, config)


__all__ = [
    "DEFAULT_CONFIG_PATHS",
    "DEFAULT_CONTROLLER_CONFIG",
    "DEFAULT_DASHBOARD_RUNTIME_CONFIG",
    "AppConfigError",
    "AppFactory",
    "AppRuntimeSpec",
    "build_dashboard_runtime",
    "build_mit_config",
    "build_process_launch_specs",
    "build_robot_controller_runtime",
    "config_path",
    "load_can_device_config",
    "load_config_paths",
    "load_dashboard_runtime_config",
    "load_processes_config",
    "load_robot_controller_config",
    "load_yaml_mapping",
    "policy_path",
    "validate_runtime_safety",
    "write_dashboard_runtime_config",
]
