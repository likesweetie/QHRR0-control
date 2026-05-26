from __future__ import annotations

from robot_controller.core.config import MitCommandShmConfig
from robot_controller.core.state import MitTarget
from robot_controller.shm.control_command import ControlCommandShm, ControlTarget


class ShmMitCommandWriter:
    def __init__(self, config: MitCommandShmConfig, *, source_id: int = 2):
        del source_id
        self.config = config
        self.writer: ControlCommandShm | None = None

    def close(self) -> None:
        if self.writer is not None:
            self.writer.close()
            self.writer = None

    def publish(self, targets: list[MitTarget]) -> None:
        if len(targets) != self.config.target_count:
            raise ValueError(
                f"Incomplete MIT command batch: targets={len(targets)}, expected={self.config.target_count}"
            )
        if self.writer is None:
            self.writer = ControlCommandShm.open_writer(self.config.name)
        self.writer.write_targets(
            [
                ControlTarget(
                    can_id=target.can_id,
                    q=target.position_rad,
                    dq=target.velocity_rad_s,
                    kp=target.kp,
                    kd=target.kd,
                    tau=target.torque_ff_nm,
                )
                for target in targets
            ]
        )
