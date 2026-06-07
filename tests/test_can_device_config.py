from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from robot_controller.config import load_config_paths
from robot_controller.config.can_device import (
    CanDeviceConfigError,
    load_can_device_config,
)


CONFIG = Path("config/app_config/can_device_config.yaml")


class CanDeviceConfigTest(unittest.TestCase):
    def test_loads_current_can_device_schema(self) -> None:
        config = load_can_device_config(CONFIG)

        self.assertEqual(config.imu.type, "e2box")
        self.assertEqual(config.imu.request_id, 0x221)
        self.assertEqual(config.imu.quat_id, 0x2A1)
        self.assertEqual(config.imu.gyro_id, 0x321)
        self.assertEqual(config.imu.cmd_get_quat, 0x01)
        self.assertTrue(config.imu.normalize_quat)

        spg = config.drivers["spg_mit"]
        self.assertEqual(spg.p_max_rad, 12.5)
        self.assertEqual(spg.v_max_rad_s, 45.0)
        self.assertEqual(spg.kp_max, 500.0)
        self.assertEqual(spg.kd_max, 5.0)
        self.assertEqual(spg.tau_max_nm, 33.0)
        self.assertEqual(spg.feedback_position_max_rad, 12.56)
        self.assertEqual(spg.iq_full_scale_count, 2048.0)
        self.assertEqual(spg.iq_full_scale_current_a, 33.0)
        self.assertEqual(spg.set_zero_hold_s, 0.020)

    def test_can_device_path_registry_entry_loads(self) -> None:
        paths = load_config_paths()
        config = load_can_device_config(paths.config("can_device"))

        self.assertIn("spg_mit", config.drivers)

    def test_unsupported_imu_type_is_rejected(self) -> None:
        path = self._write_config(self._valid_config().replace('type: "e2box"', 'type: "other"'))

        with self.assertRaisesRegex(CanDeviceConfigError, "imu.type"):
            load_can_device_config(path)

    def test_non_positive_imu_scale_is_rejected(self) -> None:
        path = self._write_config(self._valid_config().replace("quat_scale: 10000.0", "quat_scale: 0.0"))

        with self.assertRaisesRegex(CanDeviceConfigError, "quat_scale"):
            load_can_device_config(path)

    def test_negative_set_zero_hold_is_rejected(self) -> None:
        path = self._write_config(self._valid_config().replace("set_zero_hold_s: 0.020", "set_zero_hold_s: -0.1"))

        with self.assertRaisesRegex(CanDeviceConfigError, "set_zero_hold_s"):
            load_can_device_config(path)

    def test_empty_driver_key_is_rejected(self) -> None:
        path = self._write_config(self._valid_config().replace("spg_mit:", '"":'))

        with self.assertRaisesRegex(CanDeviceConfigError, "drivers key"):
            load_can_device_config(path)

    def _write_config(self, text: str) -> Path:
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        path = Path(tmpdir.name) / "can_device_config.yaml"
        path.write_text(text, encoding="utf-8")
        return path

    def _valid_config(self) -> str:
        return CONFIG.read_text(encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
