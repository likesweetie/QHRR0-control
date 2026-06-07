from __future__ import annotations

import copy
import tempfile
import unittest
from pathlib import Path

from robot_controller.config import (
    ConfigError,
    HardwareSafetyOptions,
    load_robot_controller_config,
    validate_runtime_safety,
)


CONFIG = Path("config/app_config/robot_controller.yaml")


class ConfigSafetyGateTest(unittest.TestCase):
    def test_default_config_loads_from_registry(self) -> None:
        config = load_robot_controller_config()

        self.assertEqual(config.robot_platform.robot.name, "qhrr")
        self.assertEqual(config.can_device.imu.request_id, 0x221)
        self.assertEqual(config.can.interface, "vcan0")
        self.assertEqual(config.can.bitrate, 1_000_000)
        self.assertEqual(config.can.daemon.ipc_socket_path, "/tmp/qhrr_can_daemon.sock")
        self.assertEqual(config.can.motors.can_ids, [0x141, 0x142, 0x143])
        self.assertEqual(config.shm.mit_command.target_count, len(config.robot_platform.actuators))
        self.assertEqual(config.processes[0].env_vars, {})

    def test_simulation_mode_with_vcan_passes(self) -> None:
        config = load_robot_controller_config(CONFIG)
        spg = config.can_device.drivers["spg_mit"]
        self.assertEqual(config.can.mit_protocol_range.position_rad, spg.p_max_rad)
        self.assertEqual(config.can.mit_protocol_range.velocity_rad_s, spg.v_max_rad_s)
        self.assertEqual(config.can.mit_protocol_range.kp, spg.kp_max)
        validate_runtime_safety(
            config,
            HardwareSafetyOptions(
                hardware_requested=False,
                motor_enable_confirmed=False,
                estop_ok=False,
            ),
        )

    def test_simulation_mode_with_real_can_rejects(self) -> None:
        config = copy.deepcopy(load_robot_controller_config(CONFIG))
        config.can.interface = "can0"
        with self.assertRaisesRegex(ConfigError, "simulation mode rejects real CAN"):
            validate_runtime_safety(
                config,
                HardwareSafetyOptions(False, False, False),
            )

    def test_hardware_mode_requires_explicit_flags_and_real_can(self) -> None:
        config = copy.deepcopy(load_robot_controller_config(CONFIG))
        config.runtime.mode = "hardware"
        with self.assertRaisesRegex(ConfigError, "--hardware"):
            validate_runtime_safety(
                config,
                HardwareSafetyOptions(False, False, False),
            )

        with self.assertRaisesRegex(ConfigError, "virtual CAN"):
            validate_runtime_safety(
                config,
                HardwareSafetyOptions(True, True, True),
            )

    def test_hardware_mode_requires_allow_real_can_and_estop(self) -> None:
        config = copy.deepcopy(load_robot_controller_config(CONFIG))
        config.runtime.mode = "hardware"
        config.can.interface = "can0"
        config.hardware.allow_real_can = False
        with self.assertRaisesRegex(ConfigError, "allow_real_can"):
            validate_runtime_safety(
                config,
                HardwareSafetyOptions(True, True, True),
            )

        config.hardware.allow_real_can = True
        with self.assertRaisesRegex(ConfigError, "--estop-ok"):
            validate_runtime_safety(
                config,
                HardwareSafetyOptions(True, True, False),
            )

        validate_runtime_safety(
            config,
            HardwareSafetyOptions(True, True, True),
        )

    def test_removed_root_path_keys_are_rejected(self) -> None:
        path = self._write_controller_config(
            CONFIG.read_text(encoding="utf-8")
            + "\nplatform_config: config/app_config/robot_platform.yaml\n"
        )

        with self.assertRaisesRegex(ConfigError, "platform_config"):
            load_robot_controller_config(path)

    def test_removed_can_motors_key_is_rejected(self) -> None:
        path = self._write_controller_config(
            CONFIG.read_text(encoding="utf-8").replace(
                "  imu:\n",
                "  motors: {}\n\n  imu:\n",
            )
        )

        with self.assertRaisesRegex(ConfigError, "can.motors"):
            load_robot_controller_config(path)

    def test_removed_hardware_gate_keys_are_rejected(self) -> None:
        path = self._write_controller_config(
            CONFIG.read_text(encoding="utf-8").replace(
                "  allow_real_can: false\n",
                "  allow_real_can: false\n  require_manual_arm: true\n",
            )
        )

        with self.assertRaisesRegex(ConfigError, "require_manual_arm"):
            load_robot_controller_config(path)

    def _write_controller_config(self, text: str) -> Path:
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        path = Path(tmpdir.name) / "robot_controller.yaml"
        path.write_text(text, encoding="utf-8")
        return path


if __name__ == "__main__":
    unittest.main()
