from __future__ import annotations

import unittest

from qhrr0.factory.app_factory.app_factory import AppFactory, load_dashboard_runtime_config


class RobotSpecConfigTest(unittest.TestCase):
    def test_controller_runtime_uses_qhrr0_static_spec_and_can_device(self) -> None:
        runtime_spec = AppFactory().create_robot_controller_runtime_spec(
            controller_config_path="qhrr0/config/app_config/robot_controller.yaml",
        )

        runtime = runtime_spec.controller
        actuators = runtime.actuators

        self.assertEqual([actuator.can_id for actuator in actuators], [0x141, 0x142, 0x143])
        self.assertEqual([actuator.driver for actuator in actuators], ["spg_mit"] * 3)
        self.assertEqual(runtime.imu.request_id, 0x221)
        self.assertEqual(runtime.imu.name, "e2box_imu")
        self.assertTrue(any(process.name == "dashboard" for process in runtime_spec.processes))

    def test_dashboard_runtime_is_written_as_config_file(self) -> None:
        runtime_spec = AppFactory().create_robot_controller_runtime_spec(
            controller_config_path="qhrr0/config/app_config/robot_controller.yaml",
        )
        self.assertIsNotNone(runtime_spec.dashboard)

        dashboard_process = next(process for process in runtime_spec.processes if process.name == "dashboard")
        self.assertIn("--runtime-config", dashboard_process.command)
        self.assertNotIn("DASHBOARD_RUNTIME_JSON", dashboard_process.env_vars)

        assert runtime_spec.dashboard is not None
        runtime = load_dashboard_runtime_config(runtime_spec.dashboard.runtime_config_path)

        self.assertEqual(runtime.effective_config()["can"]["iface"], "vcan0")
        self.assertEqual(len(runtime.processes), len(runtime_spec.processes))


if __name__ == "__main__":
    unittest.main()
