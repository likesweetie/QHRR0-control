from __future__ import annotations

from robot_controller.supervisor import ProcessHealth, ProcessSupervisor


class ChildProcessManager(ProcessSupervisor):
    pass


__all__ = ["ChildProcessManager", "ProcessHealth"]
