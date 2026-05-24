from __future__ import annotations

import logging
import math
import time
from collections import deque

from .utils.hal_can_bus import CANFrame

from hal.hardware.can.actuator.driver import ActuatorDriver
from hal.hardware.can.imu.driver import IMUDriver

from .QHRR0_HW.SPG_actuator.dongilC_motor_protocol import (
    SPGActuatorProtocol,
    SPGMITConfig,
)
from .QHRR0_HW.e2box_imu.e2box_imu_protocol import E2BoxIMUProtocol
from .core.config import RobotControllerConfig
from .core.robot_state_shm import RobotStateShmWriter
from .core.state import MitCommandBatch, MitTarget, RobotControllerState
from .utils.can_daemon_client import CANProcessClient
from .utils.process_supervisor import ProcessSupervisor


CAN_SFF_MASK = 0x7FF
TX_ECHO_REJECT_WINDOW_S = 0.25


logger = logging.getLogger(__name__)


class RobotControllerRuntimeIO:
    def __init__(
        self,
        config: RobotControllerConfig,
        process_supervisor: ProcessSupervisor,
    ) -> None:
        self.config = config
        self.process_supervisor = process_supervisor
        self.can_client: CANProcessClient | None = None
        self.actuator_drivers = self._make_actuator_drivers()
        self.imu_driver = self._make_imu_driver()
        self.last_safe_command_reason: str | None = None
        self.recent_tx_frames: deque[tuple[float, int, bytes]] = deque(maxlen=512)
        self.control_state_writer = RobotStateShmWriter(
            name=config.shm.control_state.name,
            size_bytes=config.shm.control_state.size_bytes,
        )
        self.dashboard_state_writer = RobotStateShmWriter(
            name=config.shm.dashboard_state.name,
            size_bytes=config.shm.dashboard_state.size_bytes,
        )
        self.last_control_state_publish_t = 0.0
        self.last_dashboard_state_publish_t = 0.0

    def connect_can_daemon(self) -> None:
        if self.can_client is not None:
            return
        daemon_config = self.config.can.daemon
        self.can_client = CANProcessClient(
            socket_path=daemon_config.ipc_socket_path,
            connect_timeout_s=daemon_config.connect_timeout_s,
        )
        self.can_client.connect()
        self._register_can_callbacks()

    def bringup_imu(self) -> None:
        imu_config = self.config.can.imu
        if not imu_config.enabled or not imu_config.request_all_on_start:
            return
        for _ in range(int(imu_config.startup_request_count)):
            self.send_can_frame(self.imu_driver.make_request_all_frame())
            time.sleep(float(imu_config.startup_request_delay_s))

    def bringup_motors(self) -> None:
        if self.config.can.motors.set_zero_on_start:
            for can_id in self.config.can.motors.can_ids:
                self.send_can_frame(self._driver_for(can_id).make_zero_position_frame())
                time.sleep(self.config.can.bringup_delay_s)

        if self.config.can.motors.enter_on_start:
            for can_id in self.config.can.motors.can_ids:
                self.send_can_frame(self._driver_for(can_id).make_enable_frame())
                time.sleep(self.config.can.bringup_delay_s)

    def request_imu_on_tick(self) -> None:
        imu_config = self.config.can.imu
        if imu_config.enabled and imu_config.request_all_each_tick:
            self.send_can_frame(self.imu_driver.make_request_all_frame())

    def send_damping_once(self, reason: str) -> None:
        if reason == self.last_safe_command_reason:
            return

        logger.warning("Sending one damping command because %s", reason)
        self.last_safe_command_reason = reason
        targets = [
            MitTarget(
                can_id=can_id,
                position_rad=0.0,
                velocity_rad_s=0.0,
                kp=0.0,
                kd=0.5,
                torque_ff_nm=0.0,
            )
            for can_id in self.config.can.motors.can_ids
        ]
        self.send_mit_batch(
            MitCommandBatch(
                source=f"safe:damping:{reason}",
                timestamp=time.time(),
                targets=targets,
            )
        )

    def validate_mit_batch(self, batch: MitCommandBatch) -> None:
        allowed_ids = set(self.config.can.motors.can_ids)
        limits = self.config.can.mit_limits
        seen_ids: set[int] = set()

        for target in batch.targets:
            if target.can_id not in allowed_ids:
                raise ValueError(f"MIT batch contains unconfigured CAN ID 0x{target.can_id:X}")
            if target.can_id in seen_ids:
                raise ValueError(f"MIT batch contains duplicate CAN ID 0x{target.can_id:X}")
            seen_ids.add(target.can_id)
            values = (
                target.position_rad,
                target.velocity_rad_s,
                target.kp,
                target.kd,
                target.torque_ff_nm,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"MIT batch contains non-finite value for CAN ID 0x{target.can_id:X}")

            for field, value, lo, hi in (
                ("position_rad", target.position_rad, -limits.position_rad, limits.position_rad),
                ("velocity_rad_s", target.velocity_rad_s, -limits.velocity_rad_s, limits.velocity_rad_s),
                ("kp", target.kp, 0.0, limits.kp),
                ("kd", target.kd, 0.0, limits.kd),
                ("torque_ff_nm", target.torque_ff_nm, -limits.torque_ff_nm, limits.torque_ff_nm),
            ):
                if value < lo or value > hi:
                    raise ValueError(
                        f"MIT {field} out of range for CAN ID 0x{target.can_id:X}: "
                        f"{value} not in [{lo}, {hi}]"
                    )

        missing_ids = allowed_ids - seen_ids
        if missing_ids:
            missing = ", ".join(f"0x{can_id:X}" for can_id in sorted(missing_ids))
            raise ValueError(f"Incomplete MIT command batch, missing CAN IDs: {missing}")

    def send_mit_batch(self, batch: MitCommandBatch) -> None:
        for target in batch.targets:
            driver = self._driver_for(target.can_id)
            self.send_can_frame(
                driver.make_impedance_command_frame(
                    position_rad=target.position_rad,
                    velocity_rad_s=target.velocity_rad_s,
                    kp=target.kp,
                    kd=target.kd,
                    torque_ff_nm=target.torque_ff_nm,
                )
            )

    def mark_mit_command_active(self) -> None:
        self.last_safe_command_reason = None

    def publish_states(self, controller_state: RobotControllerState) -> None:
        now = time.monotonic()
        self.control_state_writer.publish(self._control_state_snapshot(controller_state, now))
        self.dashboard_state_writer.publish(self._dashboard_state_snapshot(controller_state, now))
        self.last_control_state_publish_t = now
        self.last_dashboard_state_publish_t = now

    def publish_due_states(self, controller_state: RobotControllerState) -> None:
        now = time.monotonic()
        control_period_s = 1.0 / float(self.config.shm.control_state.publish_hz)
        dashboard_period_s = 1.0 / float(self.config.shm.dashboard_state.publish_hz)

        if now - self.last_control_state_publish_t >= control_period_s:
            self.control_state_writer.publish(self._control_state_snapshot(controller_state, now))
            self.last_control_state_publish_t = now

        if now - self.last_dashboard_state_publish_t >= dashboard_period_s:
            self.dashboard_state_writer.publish(self._dashboard_state_snapshot(controller_state, now))
            self.last_dashboard_state_publish_t = now

    def shutdown_actuators(self) -> None:
        if self.can_client is None:
            logger.info("Skipping actuator shutdown because CAN daemon client was not connected")
            return

        try:
            self.send_damping_once("controller shutdown")
            time.sleep(0.02)
            if self.config.can.motors.exit_on_shutdown:
                for can_id in self.config.can.motors.can_ids:
                    self.send_can_frame(self._driver_for(can_id).make_disable_frame())
                    time.sleep(self.config.can.bringup_delay_s)
        except OSError as exc:
            logger.warning("CAN error during actuator shutdown: %s", exc)
        except Exception as exc:
            logger.warning("Actuator shutdown warning: %s", exc)

    def close(self) -> None:
        self.control_state_writer.close()
        self.dashboard_state_writer.close()
        if self.can_client is not None:
            self.can_client.close()
            self.can_client = None

    def get_actuator_states(self):
        return {
            can_id: driver.get_state()
            for can_id, driver in self.actuator_drivers.items()
        }

    def get_imu_state(self):
        return self.imu_driver.get_state()

    def send_can_frame(self, frame: CANFrame) -> None:
        if frame.can_id < 0 or frame.can_id > CAN_SFF_MASK:
            raise ValueError(f"Only standard 11-bit CAN IDs are supported: 0x{frame.can_id:X}")
        self.connect_can_daemon()
        assert self.can_client is not None
        ok = self.can_client.send(
            frame,
            block=self.config.can.daemon.send_block,
            timeout=self.config.can.daemon.send_timeout_s,
        )
        if not ok:
            raise RuntimeError(f"CAN daemon TX queue is full: can_id=0x{frame.can_id:X}")
        self._remember_tx_frame(frame)

    def _control_state_snapshot(self, controller_state: RobotControllerState, now: float) -> dict:
        imu_state = self.imu_driver.get_state()
        imu_status = self.imu_driver.get_comm_status()
        return {
            "schema": "qhrr.control_state.v1",
            "timestamp_monotonic": now,
            "timestamp_unix": time.time(),
            "controller_state": controller_state.name,
            "imu": {
                "quat_xyzw": list(imu_state.quat_xyzw) if imu_state.quat_xyzw is not None else None,
                "projected_gravity_b": (
                    list(imu_state.projected_gravity_b)
                    if imu_state.projected_gravity_b is not None
                    else None
                ),
                "angular_velocity_rad_s": (
                    list(imu_state.angular_velocity_rad_s)
                    if imu_state.angular_velocity_rad_s is not None
                    else None
                ),
                "last_quat_t": imu_state.last_quat_t,
                "last_gyro_t": imu_state.last_gyro_t,
                "quat_online": bool(imu_status["quat"].is_online),
                "gyro_online": bool(imu_status["gyro"].is_online),
                "quat_stale": bool(imu_status["quat"].is_stale),
                "gyro_stale": bool(imu_status["gyro"].is_stale),
            },
            "actuators": [
                self._control_actuator_snapshot(can_id, driver, now)
                for can_id, driver in sorted(self.actuator_drivers.items())
            ],
        }

    def _dashboard_state_snapshot(self, controller_state: RobotControllerState, now: float) -> dict:
        imu_state = self.imu_driver.get_state()
        imu_status = self.imu_driver.get_comm_status()
        return {
            "schema": "qhrr.dashboard_state.v1",
            "timestamp_monotonic": now,
            "timestamp_unix": time.time(),
            "controller_state": controller_state.name,
            "can": {
                "iface": self.config.can.interface,
                "command_timeout_s": self.config.can.command_timeout_s,
            },
            "processes": self.process_supervisor.status(),
            "imu": {
                "quat_xyzw": list(imu_state.quat_xyzw) if imu_state.quat_xyzw is not None else None,
                "projected_gravity_b": (
                    list(imu_state.projected_gravity_b)
                    if imu_state.projected_gravity_b is not None
                    else None
                ),
                "angular_velocity_rad_s": (
                    list(imu_state.angular_velocity_rad_s)
                    if imu_state.angular_velocity_rad_s is not None
                    else None
                ),
                "last_quat_t": imu_state.last_quat_t,
                "last_gyro_t": imu_state.last_gyro_t,
                "quat_comm": self._freshness_to_dict(imu_status["quat"]),
                "gyro_comm": self._freshness_to_dict(imu_status["gyro"]),
            },
            "actuators": [
                self._actuator_snapshot(can_id, driver, now)
                for can_id, driver in sorted(self.actuator_drivers.items())
            ],
        }

    def _control_actuator_snapshot(self, can_id: int, driver: ActuatorDriver, now: float) -> dict:
        state = driver.get_state()
        comm = driver.get_comm_status()
        return {
            "can_id": can_id,
            "position_rad": state.position_rad,
            "velocity_rad_s": state.velocity_rad_s,
            "torque_nm": state.torque_nm,
            "current_a": state.current_a,
            "is_enabled": state.is_enabled,
            "fault_code": state.fault_code,
            "last_feedback_t": state.last_feedback_t,
            "age_s": None if state.last_feedback_t <= 0.0 else max(0.0, now - state.last_feedback_t),
            "online": bool(comm.is_online),
            "stale": bool(comm.is_stale),
        }

    def _actuator_snapshot(self, can_id: int, driver: ActuatorDriver, now: float) -> dict:
        state = driver.get_state()
        comm = driver.get_comm_status()
        return {
            "can_id": can_id,
            "name": driver.name,
            "position_rad": state.position_rad,
            "velocity_rad_s": state.velocity_rad_s,
            "torque_nm": state.torque_nm,
            "current_a": state.current_a,
            "temperature_c": state.temperature_c,
            "voltage_v": state.voltage_v,
            "fault_code": state.fault_code,
            "is_enabled": state.is_enabled,
            "mode": state.mode,
            "last_feedback_t": state.last_feedback_t,
            "age_s": None if state.last_feedback_t <= 0.0 else max(0.0, now - state.last_feedback_t),
            "raw": state.raw,
            "comm": self._freshness_to_dict(comm),
        }

    @staticmethod
    def _freshness_to_dict(status) -> dict:
        return {
            "is_online": bool(status.is_online),
            "is_stale": bool(status.is_stale),
            "rx_count": int(status.rx_count),
            "timeout_count": int(status.timeout_count),
            "decode_error_count": int(status.decode_error_count),
            "last_rx_t": float(status.last_rx_t),
            "last_fault_t": float(status.last_fault_t),
        }

    def _make_actuator_drivers(self) -> dict[int, ActuatorDriver]:
        limits = self.config.can.mit_limits
        mit_config = SPGMITConfig(
            p_max=limits.position_rad,
            v_max=limits.velocity_rad_s,
            kp_max=limits.kp,
            kd_max=limits.kd,
            tau_max=limits.torque_ff_nm,
        )
        return {
            can_id: ActuatorDriver(
                name=f"actuator_0x{can_id:03X}",
                protocol=SPGActuatorProtocol(
                    command_id=can_id,
                    feedback_id=can_id,
                    mit_config=mit_config,
                    expose_single_turn_position=True,
                ),
                feedback_timeout=self.config.can.command_timeout_s,
            )
            for can_id in self.config.can.motors.can_ids
        }

    def _make_imu_driver(self) -> IMUDriver:
        return IMUDriver(
            name="e2box_imu",
            protocol=E2BoxIMUProtocol(),
            quat_timeout=self.config.can.command_timeout_s,
            gyro_timeout=self.config.can.command_timeout_s,
        )

    def _driver_for(self, can_id: int) -> ActuatorDriver:
        driver = self.actuator_drivers.get(can_id)
        if driver is None:
            raise KeyError(f"No actuator driver configured for CAN ID 0x{can_id:X}")
        return driver

    def _register_can_callbacks(self) -> None:
        if self.can_client is None:
            raise RuntimeError("CAN daemon client must be connected before callback registration")

        for driver in self.actuator_drivers.values():
            for can_id in driver.rx_can_ids():
                self.can_client.register_callback(
                    can_id,
                    lambda frame, driver=driver: self._on_actuator_frame(driver, frame),
                )

        for can_id in self.imu_driver.rx_can_ids():
            self.can_client.register_callback(can_id, self.imu_driver.on_frame)

        registered_ids = sorted(self.can_client.dispatcher.registered_ids())
        logger.info(
            "Registered CAN RX callbacks: %s",
            ", ".join(f"0x{can_id:03X}" for can_id in registered_ids),
        )

    def _remember_tx_frame(self, frame: CANFrame) -> None:
        self.recent_tx_frames.append((time.monotonic(), int(frame.can_id), bytes(frame.data)))

    def _on_actuator_frame(self, driver: ActuatorDriver, frame: CANFrame) -> None:
        if self._is_recent_tx_echo(frame):
            logger.debug(
                "Dropped local TX echo before actuator decode: can_id=0x%03X data=%s",
                frame.can_id,
                bytes(frame.data).hex(" ").upper(),
            )
            return
        driver.on_frame(frame)

    def _is_recent_tx_echo(self, frame: CANFrame) -> bool:
        now = time.monotonic()
        while self.recent_tx_frames and now - self.recent_tx_frames[0][0] > TX_ECHO_REJECT_WINDOW_S:
            self.recent_tx_frames.popleft()

        can_id = int(frame.can_id)
        data = bytes(frame.data)
        return any(
            tx_can_id == can_id and tx_data == data
            for _tx_t, tx_can_id, tx_data in self.recent_tx_frames
        )
