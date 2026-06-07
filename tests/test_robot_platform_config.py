from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from robot_controller.platform.config import (
    RobotPlatformConfigError,
    load_robot_platform_config,
)


CONFIG = Path("config/app_config/robot_platform.yaml")


class RobotPlatformConfigTest(unittest.TestCase):
    def test_loads_current_robot_platform_schema(self) -> None:
        config = load_robot_platform_config(CONFIG)

        self.assertEqual(config.robot.name, "qhrr")
        self.assertEqual(
            config.assets.mujoco_model_path,
            "third_party/mujoco/model/one_leg_v2/qhrr0_environment.xml",
        )
        self.assertEqual(config.can.allowed_interfaces, ("vcan0", "can0"))
        self.assertEqual(len(config.actuators), 3)
        self.assertEqual(config.actuators[0].name, "RL_hip_roll")
        self.assertEqual(config.actuators[0].driver, "spg_mit")
        self.assertEqual(config.actuators[0].can_id, 0x141)

    def test_duplicate_actuator_can_id_is_rejected(self) -> None:
        path = self._write_config(
            """
robot:
  name: qhrr
assets:
  mujoco_model_path: model.xml
can:
  allowed_interfaces: [vcan0]
actuators:
  - name: joint_a
    driver: spg_mit
    can_id: "0x141"
    mujoco_joint: joint_a
    mujoco_actuator: joint_a
    sign: 1.0
    offset_rad: 0.0
  - name: joint_b
    driver: spg_mit
    can_id: "0x141"
    mujoco_joint: joint_b
    mujoco_actuator: joint_b
    sign: 1.0
    offset_rad: 0.0
"""
        )

        with self.assertRaisesRegex(RobotPlatformConfigError, "Duplicate actuator CAN ID"):
            load_robot_platform_config(path)

    def test_removed_enabled_key_is_rejected(self) -> None:
        path = self._write_config(
            """
robot:
  name: qhrr
assets:
  mujoco_model_path: model.xml
can:
  allowed_interfaces: [vcan0]
actuators:
  - name: joint_a
    enabled: true
    driver: spg_mit
    can_id: "0x141"
    mujoco_joint: joint_a
    mujoco_actuator: joint_a
    sign: 1.0
    offset_rad: 0.0
"""
        )

        with self.assertRaisesRegex(RobotPlatformConfigError, "enabled"):
            load_robot_platform_config(path)

    def test_removed_root_keys_are_rejected(self) -> None:
        path = self._write_config(
            """
robot:
  name: qhrr
robots: {}
assets:
  mujoco_model_path: model.xml
can:
  allowed_interfaces: [vcan0]
actuators:
  - name: joint_a
    driver: spg_mit
    can_id: "0x141"
    mujoco_joint: joint_a
    mujoco_actuator: joint_a
    sign: 1.0
    offset_rad: 0.0
"""
        )

        with self.assertRaisesRegex(RobotPlatformConfigError, "robots"):
            load_robot_platform_config(path)

    def test_zero_sign_is_rejected(self) -> None:
        path = self._write_config(
            """
robot:
  name: qhrr
assets:
  mujoco_model_path: model.xml
can:
  allowed_interfaces: [vcan0]
actuators:
  - name: joint_a
    driver: spg_mit
    can_id: "0x141"
    mujoco_joint: joint_a
    mujoco_actuator: joint_a
    sign: 0.0
    offset_rad: 0.0
"""
        )

        with self.assertRaisesRegex(RobotPlatformConfigError, "sign"):
            load_robot_platform_config(path)

    def _write_config(self, text: str) -> Path:
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        path = Path(tmpdir.name) / "robot_platform.yaml"
        path.write_text(text.lstrip(), encoding="utf-8")
        return path


if __name__ == "__main__":
    unittest.main()
