from __future__ import annotations

import copy
import unittest
from pathlib import Path

from robot_controller.config import HardwareSafetyOptions, validate_runtime_safety
from robot_controller.core.config import ConfigError, load_robot_controller_config


CONFIG = Path("config/app_config/robot_controller.yaml")


class ConfigSafetyGateTest(unittest.TestCase):
    def test_simulation_mode_with_vcan_passes(self) -> None:
        config = load_robot_controller_config(CONFIG)
        self.assertEqual(config.can.mit_protocol_range.position_rad, config.platform.spg_mit.p_max_rad)
        self.assertEqual(config.can.mit_protocol_range.velocity_rad_s, config.platform.spg_mit.v_max_rad_s)
        self.assertEqual(config.can.mit_protocol_range.kp, config.platform.spg_mit.kp_max)
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

    def test_hardware_mode_rejects_enable_on_start(self) -> None:
        config = copy.deepcopy(load_robot_controller_config(CONFIG))
        config.runtime.mode = "hardware"
        config.can.interface = "can0"
        config.hardware.allow_real_can = True
        config.can.motors.enter_on_start = True
        with self.assertRaisesRegex(ConfigError, "enter_on_start"):
            validate_runtime_safety(
                config,
                HardwareSafetyOptions(True, True, True),
            )


if __name__ == "__main__":
    unittest.main()
