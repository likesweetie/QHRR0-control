from __future__ import annotations

from qhrr0.app.hal.hardware.can.actuator.driver import ActuatorDriver

from .dongilc_protocol import SPGActuatorProtocol, SPGMITConfig


def create_spg_actuator_driver(
    *,
    name: str,
    can_id: int,
    mit_config: SPGMITConfig,
    feedback_timeout_s: float,
    feedback_speed_is_motor_side: bool,
    iq_count_to_amp: float | None,
) -> ActuatorDriver:
    return ActuatorDriver(
        name=str(name),
        protocol=SPGActuatorProtocol(
            command_id=int(can_id),
            feedback_id=int(can_id),
            mit_config=mit_config,
            expose_single_turn_position=True,
            feedback_speed_is_motor_side=feedback_speed_is_motor_side,
            iq_count_to_amp=iq_count_to_amp,
        ),
        feedback_timeout=feedback_timeout_s,
    )
