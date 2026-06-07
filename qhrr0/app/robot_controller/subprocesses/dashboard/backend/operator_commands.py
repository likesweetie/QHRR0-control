from __future__ import annotations

import time
from collections.abc import Iterable

from qhrr0.app.robot_controller.shm.types.operator_command import (
    OPERATOR_ZERO_TARGET_CAPACITY,
    OPERATOR_ZERO_TARGET_MAGIC,
    OperatorCommandC,
    OperatorCommandCode,
    OperatorCommandShm,
)


def build_operator_command(
    code: OperatorCommandCode | int,
    *,
    target_mask: int = 0,
    zero_targets: Iterable[tuple[int, int]] = (),
) -> OperatorCommandC:
    command = OperatorCommandC()
    command.timestamp_ns = time.time_ns()
    command.command = int(code)
    command.target_mask = int(target_mask)

    targets = tuple(zero_targets)
    if len(targets) > OPERATOR_ZERO_TARGET_CAPACITY:
        raise ValueError(
            f"zero_set target count exceeds capacity: "
            f"{len(targets)}/{OPERATOR_ZERO_TARGET_CAPACITY}"
        )
    command.zero_target_count = len(targets)
    command.zero_target_magic = OPERATOR_ZERO_TARGET_MAGIC if targets else 0
    for index, (can_id, offset_count) in enumerate(targets):
        can_id_int = int(can_id)
        offset_count_int = int(offset_count)
        if not (0 <= can_id_int <= 0x1FFFFFFF):
            raise ValueError(f"CAN ID out of range: {can_id_int}")
        if not (-32768 <= offset_count_int <= 32767):
            raise ValueError(f"MIT zero offset_count out of int16 range: {offset_count_int}")
        command.zero_targets[index].can_id = can_id_int
        command.zero_targets[index].offset_count = offset_count_int
    return command


class OperatorCommandWriter:
    def __init__(self, name: str, size_bytes: int | None = None, *, source: str = "") -> None:
        del size_bytes, source
        self.writer = OperatorCommandShm.open(name)

    def close(self) -> None:
        self.writer.close()

    def publish(
        self,
        *,
        arm: bool = False,
        clear_fault: bool = False,
        estop: bool = False,
        damping: bool = False,
        zero_set: bool = False,
        disable: bool = False,
        run: bool = False,
    ) -> int:
        if arm:
            return self.publish_code(OperatorCommandCode.ENABLE)
        if run:
            return self.publish_code(OperatorCommandCode.RUN)
        if clear_fault:
            return self.publish_code(OperatorCommandCode.RESET_FAULT)
        if estop:
            return self.publish_code(OperatorCommandCode.ESTOP)
        if damping:
            return self.publish_code(OperatorCommandCode.DAMPING)
        if zero_set:
            return self.publish_zero_set()
        if disable:
            return self.publish_code(OperatorCommandCode.DISABLE)
        return self.publish_code(OperatorCommandCode.NONE)

    def publish_code(self, code: OperatorCommandCode | int, target_mask: int = 0) -> int:
        command = build_operator_command(code, target_mask=target_mask)
        self.writer.write(command)
        return int(command.timestamp_ns)

    def publish_zero_set(self, targets: Iterable[tuple[int, int]] = ()) -> int:
        command = build_operator_command(
            OperatorCommandCode.ZERO_SET,
            zero_targets=targets,
        )
        self.writer.write(command)
        return int(command.timestamp_ns)
