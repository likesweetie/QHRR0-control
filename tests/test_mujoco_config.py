from __future__ import annotations

import unittest
from pathlib import Path

from qhrr0.factory.app_factory.app_factory import load_yaml_mapping


MUJOCO_CONFIG = Path("qhrr0/config/sim_to_sim_config/mujoco.yaml")


class MujocoConfigTest(unittest.TestCase):
    def test_removed_mujoco_can_config_is_not_supported(self) -> None:
        config = load_yaml_mapping(MUJOCO_CONFIG)

        self.assertNotIn("mujoco_can", config)
        self.assertNotIn("socketcan", config)
        self.assertNotIn("spg_mit", config)

    def test_imu_sensors_are_explicitly_named_at_root(self) -> None:
        config = load_yaml_mapping(MUJOCO_CONFIG)
        imu_sensors = config["imu_sensors"]

        self.assertIsInstance(imu_sensors["quat_sensor_name"], str)
        self.assertIsInstance(imu_sensors["gyro_sensor_name"], str)
        self.assertNotEqual(imu_sensors["quat_sensor_name"], "")
        self.assertNotEqual(imu_sensors["gyro_sensor_name"], "")

    def test_spg_zero_hold_lives_in_can_device_config(self) -> None:
        config = load_yaml_mapping(Path("qhrr0/config/robot_config/can_device_config.yaml"))
        spg_mit = config["drivers"]["spg_mit"]

        self.assertIn("set_zero_hold_s", spg_mit)
        self.assertGreaterEqual(float(spg_mit["set_zero_hold_s"]), 0.0)


if __name__ == "__main__":
    unittest.main()
