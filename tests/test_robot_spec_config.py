from __future__ import annotations

import unittest

from qhrr0_hw.robot_spec import robot_spec_from_config
from robot_controller.config.can_device import load_can_device_config
from robot_controller.platform.config import load_robot_platform_config


class RobotSpecConfigTest(unittest.TestCase):
    def test_robot_spec_uses_robot_platform_and_can_device(self) -> None:
        robot_platform = load_robot_platform_config("config/app_config/robot_platform.yaml")
        can_device = load_can_device_config("config/app_config/can_device_config.yaml")

        spec = robot_spec_from_config(robot_platform, can_device)

        self.assertEqual([actuator.can_id for actuator in spec.actuators], [0x141, 0x142, 0x143])
        self.assertEqual([actuator.driver for actuator in spec.actuators], ["spg_mit"] * 3)
        self.assertEqual(spec.actuators[0].joint_index, 0)
        self.assertEqual(spec.imu.request_id, 0x221)
        self.assertEqual(spec.imu.name, "e2box")


if __name__ == "__main__":
    unittest.main()
