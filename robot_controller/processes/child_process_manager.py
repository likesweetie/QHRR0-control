from __future__ import annotations

from dataclasses import dataclass

from robot_controller.core.config import ProcessConfig
from robot_controller.utils.process_supervisor import _ProcessSupervisorBase


@dataclass(frozen=True)
class ProcessHealth:
    can_daemon_alive: bool
    task_controller_alive: bool
    aux_reader_alive: bool
    dashboard_alive: bool


class ChildProcessManager(_ProcessSupervisorBase):
    def __init__(self, process_configs: list[ProcessConfig]):
        super().__init__(process_configs)

    def health(self) -> ProcessHealth:
        return ProcessHealth(
            can_daemon_alive=self.is_alive("can_daemon"),
            task_controller_alive=self.is_alive("task_controller"),
            aux_reader_alive=self.is_alive("aux_reader"),
            dashboard_alive=self.is_alive("dashboard"),
        )
