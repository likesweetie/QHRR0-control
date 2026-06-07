from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_controller.config.loader import (
    ConfigError,
    load_yaml_mapping,
    require_bool,
    require_int,
    require_key,
    require_list,
    require_mapping,
)


@dataclass
class ProcessConfig:
    name: str
    command: list[str]
    start_order: int
    stop_order: int
    new_terminal: bool
    terminal_command: list[str]
    working_dir: str
    env: dict[str, str]


def parse_process_config(item: Any, index: int) -> ProcessConfig:
    if not isinstance(item, dict):
        raise ConfigError(f"Config key must be a mapping: processes[{index}]")
    command = require_list(item, "command", f"processes[{index}]")
    if not command:
        raise ConfigError(f"Config key must not be empty: processes[{index}].command")
    env_raw = require_mapping(item, "env", f"processes[{index}]")
    config = ProcessConfig(
        name=str(require_key(item, "name", f"processes[{index}]")),
        command=[str(part) for part in command],
        start_order=require_int(item, "start_order", f"processes[{index}]"),
        stop_order=require_int(item, "stop_order", f"processes[{index}]"),
        new_terminal=require_bool(item, "new_terminal", f"processes[{index}]"),
        terminal_command=[
            str(part)
            for part in require_list(item, "terminal_command", f"processes[{index}]")
        ],
        working_dir=str(require_key(item, "working_dir", f"processes[{index}]")),
        env={str(key): str(value) for key, value in env_raw.items()},
    )
    validate_process_config(config)
    return config


def load_processes_config(path: str | Path) -> list[ProcessConfig]:
    raw = load_yaml_mapping(path)
    configs = [
        parse_process_config(item, index)
        for index, item in enumerate(require_list(raw, "processes", str(path)))
    ]
    validate_processes_config(configs)
    return configs


def validate_process_config(config: ProcessConfig) -> None:
    if not config.name:
        raise ConfigError("process name must not be empty")
    if not config.command:
        raise ConfigError(f"process {config.name} command must not be empty")
    if not config.working_dir:
        raise ConfigError(f"process {config.name} working_dir must not be empty")
    if config.new_terminal and not config.terminal_command:
        raise ConfigError(
            f"process {config.name} terminal_command must not be empty when new_terminal is true"
        )


def validate_processes_config(configs: list[ProcessConfig]) -> None:
    names = [process.name for process in configs]
    if len(set(names)) != len(names):
        raise ConfigError("processes must not contain duplicate names")
