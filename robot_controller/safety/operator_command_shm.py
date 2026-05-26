from __future__ import annotations

from robot_controller.shm.operator_command import (
    OperatorCommandCode,
    OperatorCommandShm,
)

from .safety_controller import OperatorCommand


class OperatorCommandShmSource:
    def __init__(self, name: str) -> None:
        self.reader = OperatorCommandShm.open_reader(name)

    def close(self) -> None:
        self.reader.close()

    def read_latest(self) -> OperatorCommand | None:
        raw = self.reader.read_new()
        if raw is None:
            return None
        try:
            code = OperatorCommandCode(int(raw.command))
        except ValueError:
            return None
        return OperatorCommand(
            arm=code == OperatorCommandCode.ENABLE,
            clear_fault=code == OperatorCommandCode.RESET_FAULT,
            estop=code == OperatorCommandCode.ESTOP,
        )


class OperatorCommandShmWriter:
    def __init__(self, name: str, size_bytes: int, *, source: str) -> None:
        del size_bytes, source
        self.writer = OperatorCommandShm.open_writer(name)

    def close(self) -> None:
        self.writer.close()

    def publish(
        self,
        *,
        arm: bool = False,
        clear_fault: bool = False,
        estop: bool = False,
    ) -> int:
        if arm:
            return self.writer.publish(OperatorCommandCode.ENABLE)
        if clear_fault:
            return self.writer.publish(OperatorCommandCode.RESET_FAULT)
        if estop:
            return self.writer.publish(OperatorCommandCode.ESTOP)
        return self.writer.publish(OperatorCommandCode.NONE)
