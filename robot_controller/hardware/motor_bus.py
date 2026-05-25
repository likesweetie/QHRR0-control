from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from hal.hardware.can.actuator.driver import ActuatorDriver

from robot_controller.QHRR0_HW.SPG_actuator.dongilC_motor_protocol import (
    SPGActuatorProtocol,
    SPGMITConfig,
)
from robot_controller.command.policy_command import JointCommandBatch
from robot_controller.core.config import RobotControllerConfig
from robot_controller.core.state import MitCommandBatch, MitTarget
from robot_controller.utils.hal_can_bus import CANFrame

from .can_transport import CanTransport


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MotorFeedbackItem:
    can_id: int
    name: str
    state: object
    comm: object
    age_s: float | None
    online: bool
    stale: bool


@dataclass(frozen=True)
class MotorFeedback:
    actuators: list[MotorFeedbackItem]

    @property
    def has_stale_feedback(self) -> bool:
        return any(item.stale for item in self.actuators)

    @property
    def stale_reason(self) -> str:
        stale_ids = [f"0x{item.can_id:X}" for item in self.actuators if item.stale]
        return "actuator feedback stale: " + ", ".join(stale_ids)


class MotorBus:
    def __init__(self, config: RobotControllerConfig, transport: CanTransport):
        self.config = config
        self.transport = transport
        self.drivers = self._make_actuator_drivers()

    def register_callbacks(self) -> None:
        for driver in self.drivers.values():
            for can_id in driver.rx_can_ids():
                self.transport.register_callback(
                    can_id,
                    lambda frame, driver=driver: self._on_actuator_frame(driver, frame),
                )

    def bringup(self) -> None:
        if self.config.can.motors.set_zero_on_start:
            for can_id in self.config.can.motors.can_ids:
                self.transport.send_frame(self._driver_for(can_id).make_zero_position_frame())
                time.sleep(self.config.can.bringup_delay_s)

        if self.config.can.motors.enter_on_start:
            self.enable_all("configured enter_on_start")

    def enable_all(self, reason: str) -> None:
        logger.warning("Sending motor enable commands because %s", reason)
        for can_id in self.config.can.motors.can_ids:
            self.transport.send_frame(self._driver_for(can_id).make_enable_frame())
            time.sleep(self.config.can.bringup_delay_s)

    def disable_all(self, reason: str) -> None:
        logger.warning("Sending motor disable commands because %s", reason)
        for can_id in self.config.can.motors.can_ids:
            self.transport.send_frame(self._driver_for(can_id).make_disable_frame())
            time.sleep(self.config.can.bringup_delay_s)

    def send_policy_mit_batch(self, command: JointCommandBatch) -> None:
        for target in command.targets:
            driver = self._driver_for(target.can_id)
            self.transport.send_frame(
                driver.make_impedance_command_frame(
                    position_rad=target.position_rad,
                    velocity_rad_s=target.velocity_rad_s,
                    kp=target.kp,
                    kd=target.kd,
                    torque_ff_nm=target.torque_ff_nm,
                )
            )

    def send_velocity_damping(self, reason: str) -> None:
        logger.warning(
            "Sending MIT velocity damping-like command because %s. "
            "This is q=0 qd=0 kp=0 kd=%.3f tau=0 and is not guaranteed "
            "hardware-safe until actuator firmware behavior is verified.",
            reason,
            self.config.safety.velocity_damping_kd,
        )
        targets = [
            MitTarget(
                can_id=can_id,
                position_rad=0.0,
                velocity_rad_s=0.0,
                kp=0.0,
                kd=self.config.safety.velocity_damping_kd,
                torque_ff_nm=0.0,
            )
            for can_id in self.config.can.motors.can_ids
        ]
        self.send_policy_mit_batch(
            MitCommandBatch(
                source=f"damping:{reason}",
                timestamp=time.time(),
                targets=targets,
            )
        )

    def shutdown(self, reason: str) -> None:
        self.send_velocity_damping(reason)
        time.sleep(0.02)
        if self.config.can.motors.exit_on_shutdown:
            self.disable_all(reason)

    def read_feedback(self, now: float | None = None) -> MotorFeedback:
        now = time.monotonic() if now is None else now
        items = []
        for can_id, driver in sorted(self.drivers.items()):
            state = driver.get_state()
            comm = driver.update_fault_flags()
            age_s = None if state.last_feedback_t <= 0.0 else max(0.0, now - state.last_feedback_t)
            items.append(
                MotorFeedbackItem(
                    can_id=can_id,
                    name=driver.name,
                    state=state,
                    comm=comm,
                    age_s=age_s,
                    online=bool(comm.is_online),
                    stale=bool(comm.is_stale),
                )
            )
        return MotorFeedback(actuators=items)

    def get_states(self):
        return {
            can_id: driver.get_state()
            for can_id, driver in self.drivers.items()
        }

    def _make_actuator_drivers(self) -> dict[int, ActuatorDriver]:
        protocol_range = self.config.can.mit_protocol_range
        mit_config = SPGMITConfig(
            p_max=protocol_range.position_rad,
            v_max=protocol_range.velocity_rad_s,
            kp_max=protocol_range.kp,
            kd_max=protocol_range.kd,
            tau_max=protocol_range.torque_ff_nm,
            feedback_position_max=protocol_range.feedback_position_rad,
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

    def _driver_for(self, can_id: int) -> ActuatorDriver:
        driver = self.drivers.get(can_id)
        if driver is None:
            raise KeyError(f"No actuator driver configured for CAN ID 0x{can_id:X}")
        return driver

    def _on_actuator_frame(self, driver: ActuatorDriver, frame: CANFrame) -> None:
        if self.transport.is_recent_tx_echo(frame):
            logger.debug(
                "Dropped local TX echo before actuator decode: can_id=0x%03X data=%s",
                frame.can_id,
                bytes(frame.data).hex(" ").upper(),
            )
            return
        driver.on_frame(frame)
