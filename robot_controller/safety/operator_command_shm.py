from __future__ import annotations

import time

from robot_controller.core.robot_state_shm import RobotStateShmReader, RobotStateShmWriter

from .safety_controller import OperatorCommand


OPERATOR_COMMAND_SCHEMA = "qhrr.operator_command.v1"


class OperatorCommandShmSource:
    def __init__(self, name: str) -> None:
        self.name = name
        self.reader = RobotStateShmReader(name)
        self._last_command_id: int | None = None

    def close(self) -> None:
        self.reader.close()

    def read_latest(self) -> OperatorCommand | None:
        payload = self.reader.read_latest()
        if payload is None:
            return None
        if payload.get("schema") != OPERATOR_COMMAND_SCHEMA:
            raise RuntimeError(
                f"Operator command SHM schema mismatch: {payload.get('schema')!r}"
            )
        try:
            command_id = int(payload["command_id"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"Operator command SHM is missing command_id: {exc}") from exc

        if command_id == self._last_command_id:
            return None
        self._last_command_id = command_id

        command = OperatorCommand(
            arm=bool(payload.get("arm", False)),
            clear_fault=bool(payload.get("clear_fault", False)),
            estop=bool(payload.get("estop", False)),
        )
        if command.arm or command.clear_fault or command.estop:
            return command
        return None


class OperatorCommandShmWriter:
    def __init__(self, name: str, size_bytes: int, *, source: str) -> None:
        self.writer = RobotStateShmWriter(name, size_bytes)
        self.source = source
        self._last_command_id = 0

    def close(self) -> None:
        self.writer.close()

    def publish(
        self,
        *,
        arm: bool = False,
        clear_fault: bool = False,
        estop: bool = False,
    ) -> int:
        command_id = max(time.time_ns(), self._last_command_id + 1)
        self._last_command_id = command_id
        self.writer.publish(
            {
                "schema": OPERATOR_COMMAND_SCHEMA,
                "timestamp_monotonic": time.monotonic(),
                "timestamp_unix": time.time(),
                "command_id": command_id,
                "source": self.source,
                "arm": bool(arm),
                "clear_fault": bool(clear_fault),
                "estop": bool(estop),
            }
        )
        return command_id
