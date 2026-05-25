from __future__ import annotations

import unittest
from pathlib import Path

from robot_controller.core.platform_config import load_yaml_mapping


class MujocoConfigTest(unittest.TestCase):
    def test_periodic_actuator_feedback_config_is_not_supported(self) -> None:
        config = load_yaml_mapping(Path("config/app_config/mujoco.yaml"))
        spg_mit = config["mujoco_can"]["spg_mit"]
        self.assertNotIn("periodic_feedback", spg_mit)
        self.assertNotIn("periodic_feedback_s", spg_mit)

    def test_mit_command_timeout_config_is_not_supported(self) -> None:
        config = load_yaml_mapping(Path("config/app_config/mujoco.yaml"))
        mujoco_can = config["mujoco_can"]
        self.assertNotIn("command_timeout_s", mujoco_can)


if __name__ == "__main__":
    unittest.main()
