from __future__ import annotations

from multiprocessing import shared_memory

from qhrr0.app.robot_controller.shm.types.aux_command import AuxCommandShm
from qhrr0.app.robot_controller.shm.types.control_command import ControlCommandShm
from qhrr0.app.robot_controller.shm.types.operator_command import OperatorCommandShm
from qhrr0.app.robot_controller.shm.types.robot_state import RobotStateShm


class ShmManager:
    def __init__(
        self,
        *,
        mit_command_name: str,
        aux_command_name: str,
        aux_command_size_bytes: int,
        operator_command_name: str,
        operator_command_size_bytes: int,
        control_state_name: str,
        control_state_size_bytes: int,
        dashboard_state_name: str,
        dashboard_state_size_bytes: int,
    ):
        self.mit_command_name = str(mit_command_name)
        self.aux_command_name = str(aux_command_name)
        self.aux_command_size_bytes = int(aux_command_size_bytes)
        self.operator_command_name = str(operator_command_name)
        self.operator_command_size_bytes = int(operator_command_size_bytes)
        self.control_state_name = str(control_state_name)
        self.control_state_size_bytes = int(control_state_size_bytes)
        self.dashboard_state_name = str(dashboard_state_name)
        self.dashboard_state_size_bytes = int(dashboard_state_size_bytes)
        self._segments: dict[str, shared_memory.SharedMemory] = {}

    def cleanup_stale(self) -> None:
        for name in self._segment_names():
            try:
                stale = shared_memory.SharedMemory(name=name, create=False)
            except FileNotFoundError:
                continue
            stale.close()
            stale.unlink()

    def create_all(self) -> None:
        control_command = ControlCommandShm.create(self.mit_command_name)
        self._segments[self.mit_command_name] = control_command.shm

        aux_command = AuxCommandShm.create(
            self.aux_command_name,
            size=self.aux_command_size_bytes,
        )
        self._segments[self.aux_command_name] = aux_command.shm

        operator_command = OperatorCommandShm.create(
            self.operator_command_name,
            size=self.operator_command_size_bytes,
        )
        self._segments[self.operator_command_name] = operator_command.shm

        control_state = RobotStateShm.create(
            name=self.control_state_name,
            size=self.control_state_size_bytes,
        )
        self._segments[self.control_state_name] = control_state.shm

        dashboard_state = RobotStateShm.create(
            name=self.dashboard_state_name,
            size=self.dashboard_state_size_bytes,
        )
        self._segments[self.dashboard_state_name] = dashboard_state.shm

    def close_all(self) -> None:
        for segment in self._segments.values():
            segment.close()
        self._segments.clear()

    def unlink_all(self) -> None:
        for name in self._segment_names():
            try:
                segment = shared_memory.SharedMemory(name=name, create=False)
            except FileNotFoundError:
                continue
            segment.close()
            segment.unlink()

    def _segment_names(self) -> tuple[str, ...]:
        return (
            self.mit_command_name,
            self.aux_command_name,
            self.operator_command_name,
            self.control_state_name,
            self.dashboard_state_name,
        )
