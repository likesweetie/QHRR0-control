from __future__ import annotations

import math
import socket
import struct
import time

from .config import RobotControllerConfig
from .process_supervisor import ProcessSupervisor
from .shm_command_router import ShmMitCommandRouter
from .shm_manager import ShmManager
from .state import MitCommandBatch, MitTarget, RobotControllerState


CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_SFF_MASK = 0x7FF

SPG_CMD_MIT_CONTROL = 0xC0
SPG_CMD_MIT_ENTER = 0xC1
SPG_CMD_MIT_EXIT = 0xC2
SPG_CMD_MIT_SET_ZERO = 0xC3


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def float_to_uint(value: float, value_min: float, value_max: float, bits: int) -> int:
    value = clamp(value, value_min, value_max)
    span = value_max - value_min
    max_int = (1 << bits) - 1
    return int(round((value - value_min) * max_int / span))


def build_can_frame(can_id: int, data: bytes) -> bytes:
    if len(data) > 8:
        raise ValueError("Classical CAN payload must be <= 8 bytes")
    if can_id < 0 or can_id > CAN_SFF_MASK:
        raise ValueError(f"Only standard 11-bit CAN IDs are supported: 0x{can_id:X}")
    padded = data + b"\x00" * (8 - len(data))
    return struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, len(data), padded)


class RobotController:
    def __init__(self, config: RobotControllerConfig):
        self.config = config
        self.state = RobotControllerState.CREATED
        self.shm_manager = ShmManager(config.shm)
        self.process_supervisor = ProcessSupervisor(config.processes)
        self.command_router = ShmMitCommandRouter(config.shm.mit_command)
        self._running = False
        self._sock: socket.socket | None = None

    def start(self) -> None:
        try:
            self.state = RobotControllerState.INIT_SHM
            if self.config.shm.cleanup_stale_on_start:
                self.shm_manager.cleanup_stale()
            self.shm_manager.create_all()

            self.state = RobotControllerState.START_CAN_DAEMON
            self._open_can_if_needed()
            self.process_supervisor.start_by_name(self.config.can.daemon_process)

            self.state = RobotControllerState.BRINGUP_MOTORS
            self._bringup_motors()

            self.state = RobotControllerState.START_CHILD_PROCESSES
            self.process_supervisor.start_all_except(self.config.can.daemon_process)

            self.state = RobotControllerState.RUNNING
            self._running = True
        except Exception:
            self.state = RobotControllerState.ERROR
            self.shutdown()
            raise

    def run(self) -> None:
        period_s = 1.0 / max(1.0, float(self.config.robot_controller.control_hz))
        next_t = time.perf_counter()
        while self._running:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)

            self.tick()

            next_t += period_s
            now = time.perf_counter()
            if next_t < now:
                next_t = now + period_s

    def tick(self) -> None:
        batch = self.command_router.read_latest_batch()
        if batch is None:
            self._send_zero_or_damping_command()
            return

        if not self.command_router.is_fresh(batch, self.config.can.command_timeout_s):
            self._send_zero_or_damping_command()
            return

        sanitized = self._sanitize_mit_batch(batch)
        if not sanitized.targets:
            self._send_zero_or_damping_command()
            return

        self._send_mit_batch_to_can_daemon(sanitized)

    def shutdown(self) -> None:
        if self.state == RobotControllerState.STOPPED:
            return

        self.state = RobotControllerState.SHUTTING_DOWN
        self._running = False
        try:
            self._shutdown_motors()
        except Exception as exc:
            print(f"[robot_controller] motor shutdown warning: {exc}")

        self.process_supervisor.stop_all(self.config.robot_controller.shutdown_timeout_s)
        self.command_router.close()
        self.shm_manager.close_all()
        if self.config.shm.unlink_on_shutdown:
            self.shm_manager.unlink_all()
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self.state = RobotControllerState.STOPPED

    def _bringup_motors(self) -> None:
        if self.config.can.motors.set_zero_on_start:
            for can_id in self.config.can.motors.ids:
                self._send_motor_opcode(can_id, SPG_CMD_MIT_SET_ZERO)
                time.sleep(self.config.can.bringup_delay_s)

        if self.config.can.motors.enter_on_start:
            for can_id in self.config.can.motors.ids:
                self._send_motor_opcode(can_id, SPG_CMD_MIT_ENTER)
                time.sleep(self.config.can.bringup_delay_s)

    def _shutdown_motors(self) -> None:
        try:
            self._send_zero_or_damping_command()
            time.sleep(0.02)
            if self.config.can.motors.exit_on_shutdown:
                for can_id in self.config.can.motors.ids:
                    self._send_motor_opcode(can_id, SPG_CMD_MIT_EXIT)
                    time.sleep(self.config.can.bringup_delay_s)
        except OSError:
            pass

    def _send_zero_or_damping_command(self) -> None:
        targets = [
            MitTarget(
                motor_id=can_id,
                position_rad=0.0,
                velocity_rad_s=0.0,
                kp=0.0,
                kd=min(0.5, self.config.can.mit_limits.kd),
                torque_ff_nm=0.0,
            )
            for can_id in self.config.can.motors.ids
        ]
        self._send_mit_batch_to_can_daemon(
            MitCommandBatch(
                source="fallback:damping",
                timestamp=time.time(),
                targets=targets,
            )
        )

    def _sanitize_mit_batch(self, batch: MitCommandBatch) -> MitCommandBatch:
        allowed_ids = set(self.config.can.motors.ids)
        limits = self.config.can.mit_limits
        targets: list[MitTarget] = []

        for target in batch.targets:
            if target.motor_id not in allowed_ids:
                continue
            values = (
                target.position_rad,
                target.velocity_rad_s,
                target.kp,
                target.kd,
                target.torque_ff_nm,
            )
            if not all(math.isfinite(value) for value in values):
                continue

            targets.append(
                MitTarget(
                    motor_id=target.motor_id,
                    position_rad=clamp(target.position_rad, -limits.position_rad, limits.position_rad),
                    velocity_rad_s=clamp(target.velocity_rad_s, -limits.velocity_rad_s, limits.velocity_rad_s),
                    kp=clamp(target.kp, 0.0, limits.kp),
                    kd=clamp(target.kd, 0.0, limits.kd),
                    torque_ff_nm=clamp(target.torque_ff_nm, -limits.torque_ff_nm, limits.torque_ff_nm),
                )
            )

        return MitCommandBatch(
            source=batch.source,
            timestamp=batch.timestamp,
            targets=targets,
        )

    def _send_mit_batch_to_can_daemon(self, batch: MitCommandBatch) -> None:
        for target in batch.targets:
            self._send_can(
                target.motor_id,
                self._pack_mit_payload(target),
            )

    def _send_motor_opcode(self, can_id: int, opcode: int) -> None:
        self._send_can(can_id, bytes([opcode]) + b"\x00" * 7)

    def _pack_mit_payload(self, target: MitTarget) -> bytes:
        limits = self.config.can.mit_limits
        p_u = float_to_uint(target.position_rad, -limits.position_rad, limits.position_rad, 16)
        v_u = float_to_uint(target.velocity_rad_s, -limits.velocity_rad_s, limits.velocity_rad_s, 12)
        kp_u = float_to_uint(target.kp, 0.0, limits.kp, 12)
        kd_u = float_to_uint(target.kd, 0.0, limits.kd, 8)
        t_u = float_to_uint(target.torque_ff_nm, -limits.torque_ff_nm, limits.torque_ff_nm, 8)

        data = bytearray(8)
        data[0] = SPG_CMD_MIT_CONTROL
        data[1] = (p_u >> 8) & 0xFF
        data[2] = p_u & 0xFF
        data[3] = (v_u >> 4) & 0xFF
        data[4] = ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)
        data[5] = kp_u & 0xFF
        data[6] = kd_u & 0xFF
        data[7] = t_u & 0xFF
        return bytes(data)

    def _send_can(self, can_id: int, data: bytes) -> None:
        if not self.config.can.direct_socketcan:
            # TODO: route through CAN daemon IPC when that API is finalized.
            return
        self._open_can_if_needed()
        assert self._sock is not None
        sent = self._sock.send(build_can_frame(can_id, data))
        if sent != CAN_FRAME_SIZE:
            raise OSError(f"Incomplete CAN frame write: {sent}/{CAN_FRAME_SIZE} bytes")

    def _open_can_if_needed(self) -> None:
        if not self.config.can.direct_socketcan or self._sock is not None:
            return
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((self.config.can.interface,))
        self._sock = sock
