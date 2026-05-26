from __future__ import annotations

import unittest

import robot_controller.process.can_daemon.main as can_daemon_main


class CanDaemonGateTest(unittest.TestCase):
    def test_can_daemon_module_has_no_product_specific_enable_gate(self) -> None:
        self.assertFalse(hasattr(can_daemon_main, "motor_enable_block_reason"))
        self.assertFalse(hasattr(can_daemon_main, "SPG_CMD_MIT_ENTER"))


if __name__ == "__main__":
    unittest.main()
