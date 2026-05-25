from __future__ import annotations

import math

from robot_controller.core.config import MitProtocolRangeConfig

from .policy_command import JointCommandBatch, PolicyCommand


class CommandValidator:
    def __init__(self, *, expected_can_ids: list[int], protocol_range: MitProtocolRangeConfig):
        self.expected_can_ids = [int(can_id) for can_id in expected_can_ids]
        self.protocol_range = protocol_range

    def validate(self, command: PolicyCommand) -> JointCommandBatch:
        target_ids = [int(target.can_id) for target in command.targets]
        seen_ids: set[int] = set()
        for can_id in target_ids:
            if can_id in seen_ids:
                raise ValueError(f"Policy command contains duplicate CAN ID 0x{can_id:X}")
            seen_ids.add(can_id)
            if can_id not in self.expected_can_ids:
                raise ValueError(f"Policy command contains unknown CAN ID 0x{can_id:X}")

        missing = set(self.expected_can_ids) - seen_ids
        if missing:
            formatted = ", ".join(f"0x{can_id:X}" for can_id in sorted(missing))
            raise ValueError(f"Policy command is missing actuator CAN IDs: {formatted}")

        if target_ids != self.expected_can_ids:
            expected = ", ".join(f"0x{can_id:X}" for can_id in self.expected_can_ids)
            actual = ", ".join(f"0x{can_id:X}" for can_id in target_ids)
            raise ValueError(f"Policy command CAN ID order mismatch: expected [{expected}], got [{actual}]")

        for target in command.targets:
            can_id = int(target.can_id)
            self._validate_target_values(can_id, target)

        return command

    def _validate_target_values(self, can_id: int, target) -> None:
        values = (
            target.position_rad,
            target.velocity_rad_s,
            target.kp,
            target.kd,
            target.torque_ff_nm,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"Policy command contains non-finite value for CAN ID 0x{can_id:X}")

        protocol_range = self.protocol_range
        for field, value, lo, hi in (
            ("position_rad", target.position_rad, -protocol_range.position_rad, protocol_range.position_rad),
            ("velocity_rad_s", target.velocity_rad_s, -protocol_range.velocity_rad_s, protocol_range.velocity_rad_s),
            ("kp", target.kp, 0.0, protocol_range.kp),
            ("kd", target.kd, 0.0, protocol_range.kd),
            ("torque_ff_nm", target.torque_ff_nm, -protocol_range.torque_ff_nm, protocol_range.torque_ff_nm),
        ):
            if value < lo or value > hi:
                raise ValueError(
                    f"Policy command {field} out of range for CAN ID 0x{can_id:X}: "
                    f"{value} not in [{lo}, {hi}]"
                )
