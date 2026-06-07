from __future__ import annotations

from typing import Any, Mapping

from qhrr0.app.hal.driver.actuators import SPGMITConfig
from qhrr0.factory.robot_factory.base.robot_base import Robot
from qhrr0.qhrr0_spec import QHRR0

from .app_validation_rules import AppConfigError, require_robot_matches_config, value
from .schema import (
    CanClientRuntimeConfig,
    ControllerSafetyConfig,
    ControllerTimingConfig,
    ImuRuntimeConfig,
    RobotControllerActuator,
    RobotControllerRuntimeConfig,
    ShmRuntimeConfig,
)


def build_robot_controller_runtime(
    config: Mapping[str, Any],
    robot: Robot = QHRR0,
) -> RobotControllerRuntimeConfig:
    require_robot_matches_config(robot, config)
    spg = value(config, "can_device.drivers.spg_mit")
    imu = value(config, "can_device.imu")
    iq_full_scale_count = float(spg["iq_full_scale_count"])
    if iq_full_scale_count <= 0.0:
        raise AppConfigError("can_device.drivers.spg_mit.iq_full_scale_count must be > 0")

    return RobotControllerRuntimeConfig(
        actuators=tuple(
            RobotControllerActuator(
                name=str(actuator.name),
                driver=str(actuator.driver),
                can_id=int(actuator.can_id),
            )
            for actuator in robot.actuators
        ),
        mit=build_mit_config(config),
        iq_count_to_amp=float(spg["iq_full_scale_current_a"]) / iq_full_scale_count,
        can=CanClientRuntimeConfig(
            ipc_socket_path=str(value(config, "can.daemon.ipc_socket_path")),
            connect_timeout_s=float(value(config, "can.daemon.connect_timeout_s")),
            command_timeout_s=float(value(config, "can.command_timeout_s")),
        ),
        imu=ImuRuntimeConfig(
            name=str(robot.imu.name),
            request_id=int(imu["request_id"]),
            quat_id=int(imu["quat_id"]),
            gyro_id=int(imu["gyro_id"]),
            cmd_get_quat=int(imu["cmd_get_quat"]),
            cmd_get_gyro=int(imu["cmd_get_gyro"]),
            cmd_get_all=int(imu["cmd_get_all"]),
            quat_scale=float(imu["quat_scale"]),
            gyro_scale=float(imu["gyro_scale"]),
            normalize_quat=bool(imu["normalize_quat"]),
            enabled=bool(value(config, "can.imu.enabled")),
            request_all_on_start=bool(value(config, "can.imu.request_all_on_start")),
            request_all_each_tick=bool(value(config, "can.imu.request_all_each_tick")),
            startup_request_count=int(value(config, "can.imu.startup_request_count")),
            startup_request_delay_s=float(value(config, "can.imu.startup_request_delay_s")),
        ),
        shm=ShmRuntimeConfig(
            mit_command_name=str(value(config, "shm.mit_command.name")),
            aux_command_name=str(value(config, "shm.aux_command.name")),
            aux_command_size_bytes=int(value(config, "shm.aux_command.size_bytes")),
            operator_command_name=str(value(config, "shm.operator_command.name")),
            operator_command_size_bytes=int(value(config, "shm.operator_command.size_bytes")),
            control_state_name=str(value(config, "shm.control_state.name")),
            control_state_size_bytes=int(value(config, "shm.control_state.size_bytes")),
            control_state_publish_hz=float(value(config, "shm.control_state.publish_hz")),
            dashboard_state_name=str(value(config, "shm.dashboard_state.name")),
            dashboard_state_size_bytes=int(value(config, "shm.dashboard_state.size_bytes")),
            dashboard_state_publish_hz=float(value(config, "shm.dashboard_state.publish_hz")),
        ),
        safety=ControllerSafetyConfig(
            velocity_damping_kd=float(value(config, "safety.velocity_damping_kd")),
        ),
        timing=ControllerTimingConfig(
            control_hz=float(value(config, "robot_controller.control_hz")),
            shutdown_timeout_s=float(value(config, "robot_controller.shutdown_timeout_s")),
            enable_duration_s=float(value(config, "state_machine.enable_duration_s")),
        ),
    )


def build_mit_config(config: Mapping[str, Any]) -> SPGMITConfig:
    protocol = value(config, "can.mit_protocol_range")
    return SPGMITConfig(
        p_max=float(protocol["position_rad"]),
        v_max=float(protocol["velocity_rad_s"]),
        kp_max=float(protocol["kp"]),
        kd_max=float(protocol["kd"]),
        tau_max=float(protocol["torque_ff_nm"]),
        feedback_position_max=float(protocol["feedback_position_rad"]),
    )
