from __future__ import annotations

import time

from robot_controller.core.config import MitCommandShmConfig
from robot_controller.shm.control_command import ControlCommandShm

from .policy_command import (
    CommandReadResult,
    CommandReadStatus,
    JointCommand,
    PolicyCommand,
)


class ShmPolicyCommandSource:
    def __init__(self, config: MitCommandShmConfig):
        self.config = config
        self.reader: ControlCommandShm | None = None

    def close(self) -> None:
        if self.reader is not None:
            self.reader.close()
            self.reader = None

    def read_latest(self) -> CommandReadResult:
        try:
            command = self._read_latest()
        except FileNotFoundError as exc:
            return CommandReadResult(None, CommandReadStatus.SHM_UNAVAILABLE, str(exc), None)
        except ValueError as exc:
            return CommandReadResult(None, CommandReadStatus.BAD_FORMAT, str(exc), None)
        if command is None:
            return CommandReadResult(None, CommandReadStatus.NO_COMMAND, "control command SHM is empty", None)
        return CommandReadResult(command, CommandReadStatus.AVAILABLE, "ok", command.timestamp)

    def is_fresh(self, result: CommandReadResult, timeout_s: float) -> bool:
        if result.command is None or result.timestamp is None:
            return False
        return (time.time() - result.timestamp) <= timeout_s

    def _read_latest(self) -> PolicyCommand | None:
        if self.reader is None:
            self.reader = ControlCommandShm.open_reader(self.config.name)
        raw = self.reader.read_relaxed()
        if int(raw.timestamp_ns) == 0 or int(raw.num_targets) == 0:
            return None
        target_count = min(int(raw.num_targets), len(raw.targets))
        targets = [
            JointCommand(
                can_id=int(target.can_id),
                position_rad=float(target.q),
                velocity_rad_s=float(target.dq),
                kp=float(target.kp),
                kd=float(target.kd),
                torque_ff_nm=float(target.tau),
            )
            for target in raw.targets[:target_count]
        ]
        return PolicyCommand(
            source="control_command_shm",
            timestamp=int(raw.timestamp_ns) / 1_000_000_000.0,
            targets=targets,
        )
