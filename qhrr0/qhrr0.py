# qhrr0.py
from __future__ import annotations

from factory.robot_factory.robot_factory import RobotFactory


_factory = RobotFactory()


QHRR0_CONTROLLER = _factory.create_controller(
    name="qhrr0_policy_controller",
    controller_type="policy_runner",
    required_validation_rules=(
        "controller",
        "actuator_drivers_known",
        "single_leg_3dof_layout",
        "policy_controller_requires_3_actuators",
    ),
    metadata={
        "allowed_actuator_drivers": ("spg_mit",),
        "controlled_actuators": (
            "RL_hip_roll",
            "RL_hip_pitch",
            "RL_knee_pitch",
        ),
        "action_dim": 3,
    },
    description="Static controller profile for QHRR0 one-leg policy runner.",
)


QHRR0 = _factory.create_robot(
    name="qhrr0",
    description="QHRR0 single-leg robot definition.",

    can_interfaces=(
        "vcan0",
        "can0",
    ),

    required_validation_rules=(
        "robot_identity",
        "can_interfaces",
        "actuators",
        "imu",
        "can_id_conflicts",
    ),

    controller=QHRR0_CONTROLLER,

    # Current IMU data model has one representative CAN ID.
    # 0x221 is used here as the E2Box request/representative ID.
    # If the IMU protocol later requires request_id / quat_id / gyro_id,
    # expand IMU instead of overloading can_id.
    imu=_factory.create_imu(
        name="e2box_imu",
        driver="e2box",
        can_id=0x221,
    ),

    actuators=(
        _factory.create_actuator(
            name="RL_hip_roll",
            driver="spg_mit",
            can_id=0x141,
            sign=1.0,
            offset_rad=0.0,
            group="hip_roll",
        ),
        _factory.create_actuator(
            name="RL_hip_pitch",
            driver="spg_mit",
            can_id=0x142,
            sign=1.0,
            offset_rad=0.0,
            group="hip_pitch",
        ),
        _factory.create_actuator(
            name="RL_knee_pitch",
            driver="spg_mit",
            can_id=0x143,
            sign=1.0,
            offset_rad=0.0,
            group="knee_pitch",
        ),
    ),
)


__all__ = [
    "QHRR0",
    "QHRR0_CONTROLLER",
]