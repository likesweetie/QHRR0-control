from __future__ import annotations

import unittest

from robot_controller.subprocesses.dashboard.backend.can_decode import SPG_CMD_MIT_ENTER, SPG_CMD_MIT_EXIT
from robot_controller.subprocesses.dashboard.backend.command_api import CommandError, CommandService
from robot_controller.subprocesses.dashboard.backend.state import MonitorState


class FakeCANClient:
    def __init__(self) -> None:
        self.connected = False
        self.sent = []

    def connect(self) -> None:
        self.connected = True

    def send(self, frame) -> None:
        self.sent.append(frame)


def monitor_state() -> MonitorState:
    return MonitorState(
        iface="vcan0",
        bitrate=1_000_000,
        bus_window_s=1.0,
        heartbeat_window_s=1.0,
        node_timeout_s=0.25,
        stuff_factor=1.15,
        feedback_position_max_rad=12.56,
        iq_full_scale_count=2048.0,
        iq_full_scale_current_a=33.0,
        mit_p_max_rad=12.5,
        mit_v_max_rad_s=45.0,
        mit_kp_max=500.0,
        mit_kd_max=5.0,
        mit_tau_max_nm=33.0,
        imu_request_id=0x221,
        imu_quat_id=0x2A1,
        imu_gyro_id=0x321,
        imu_cmd_get_all=0x03,
        imu_quat_scale=10000.0,
        imu_gyro_scale=100.0,
        imu_normalize_quat=True,
        actuator_configs=({"name": "RL_hip_roll", "can_id": 0x141},),
        tx_enabled=True,
        allow_actuator_commands=True,
    )


class DashboardCommandApiTest(unittest.TestCase):
    def test_blocked_controller_state_blocks_motor_enable(self) -> None:
        client = FakeCANClient()
        service = CommandService(
            monitor_state(),
            client,
            controller_safety_state_provider=lambda: "DISARMED",
        )

        with self.assertRaisesRegex(CommandError, "blocked"):
            service.send_raw(0x141, bytes([SPG_CMD_MIT_ENTER]) + b"\x00" * 7)

        self.assertEqual(client.sent, [])

    def test_blocked_controller_state_allows_motor_disable(self) -> None:
        client = FakeCANClient()
        service = CommandService(
            monitor_state(),
            client,
            controller_safety_state_provider=lambda: "DISARMED",
        )

        service.send_raw(0x141, bytes([SPG_CMD_MIT_EXIT]) + b"\x00" * 7)

        self.assertEqual(len(client.sent), 1)

    def test_unavailable_controller_state_blocks_motor_enable(self) -> None:
        client = FakeCANClient()
        service = CommandService(
            monitor_state(),
            client,
            controller_safety_state_provider=lambda: None,
        )

        with self.assertRaisesRegex(CommandError, "unavailable"):
            service.send_raw(0x141, bytes([SPG_CMD_MIT_ENTER]) + b"\x00" * 7)

    def test_damping_state_allows_motor_enable_after_arm(self) -> None:
        client = FakeCANClient()
        service = CommandService(
            monitor_state(),
            client,
            controller_safety_state_provider=lambda: "DAMPING",
            controller_safety_reason_provider=lambda: "no command",
        )

        service.send_raw(0x141, bytes([SPG_CMD_MIT_ENTER]) + b"\x00" * 7)

        self.assertEqual(len(client.sent), 1)

    def test_operator_estop_damping_blocks_motor_enable(self) -> None:
        client = FakeCANClient()
        service = CommandService(
            monitor_state(),
            client,
            controller_safety_state_provider=lambda: "DAMPING",
            controller_safety_reason_provider=lambda: "operator E-stop",
        )

        with self.assertRaisesRegex(CommandError, "E-stop damping"):
            service.send_raw(0x141, bytes([SPG_CMD_MIT_ENTER]) + b"\x00" * 7)

        self.assertEqual(client.sent, [])


if __name__ == "__main__":
    unittest.main()
