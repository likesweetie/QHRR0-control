from __future__ import annotations

import subprocess
from dataclasses import asdict

from .config import ProcessConfig


class ProcessSupervisor:
    def __init__(self, process_configs: list[ProcessConfig]):
        self.process_configs = {
            config.name: config
            for config in process_configs
            if config.enabled
        }
        self.processes: dict[str, subprocess.Popen] = {}

    def start_by_name(self, name: str) -> None:
        if not name:
            return
        config = self.process_configs.get(name)
        if config is None:
            raise KeyError(f"Unknown process: {name}")
        if self.is_alive(name):
            return
        if not config.command:
            raise ValueError(f"Process {name} has empty command")
        self.processes[name] = subprocess.Popen(config.command)

    def start_all_except(self, excluded_name: str) -> None:
        configs = sorted(
            self.process_configs.values(),
            key=lambda item: item.start_order,
        )
        for config in configs:
            if config.name == excluded_name:
                continue
            self.start_by_name(config.name)

    def stop_by_name(self, name: str, timeout_s: float = 2.0) -> None:
        process = self.processes.get(name)
        if process is None:
            return
        if process.poll() is not None:
            return

        process.terminate()
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=timeout_s)

    def stop_all(self, timeout_s: float = 2.0) -> None:
        names = sorted(
            self.process_configs,
            key=lambda name: self.process_configs[name].stop_order,
        )
        for name in names:
            self.stop_by_name(name, timeout_s=timeout_s)

    def is_alive(self, name: str) -> bool:
        process = self.processes.get(name)
        return process is not None and process.poll() is None

    def status(self) -> dict[str, object]:
        return {
            name: {
                "config": asdict(config),
                "pid": self.processes[name].pid if name in self.processes else None,
                "alive": self.is_alive(name),
                "returncode": self.processes[name].poll() if name in self.processes else None,
            }
            for name, config in self.process_configs.items()
        }
