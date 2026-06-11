from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from qhrr0.factory.app_factory.app_factory import (
    AppConfigError,
    config_path,
    load_config_paths,
    policy_path,
)


class ConfigPathRegistryTest(unittest.TestCase):
    def test_default_config_paths_resolve_from_project_root(self) -> None:
        paths = load_config_paths()

        self.assertEqual(
            config_path(paths, "robot_controller"),
            Path("qhrr0/config/app_config/robot_controller.yaml").resolve(),
        )
        self.assertEqual(
            config_path(paths, "can_device"),
            Path("qhrr0/config/robot_config/can_device_config.yaml").resolve(),
        )
        self.assertEqual(
            policy_path(paths, "policy_list"),
            Path(
                "qhrr0/app/robot_controller/subprocesses/task_controller/"
                "policy_config/qhrr/policy_list.yaml"
            ).resolve(),
        )
        self.assertTrue(config_path(paths, "policy_runner").exists())

    def test_unknown_keys_raise_config_error(self) -> None:
        paths = load_config_paths()

        with self.assertRaisesRegex(AppConfigError, "configs.missing"):
            config_path(paths, "missing")
        with self.assertRaisesRegex(AppConfigError, "policy.missing"):
            policy_path(paths, "missing")

    def test_empty_path_values_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config_dir = root / "config" / "app_config"
            config_dir.mkdir(parents=True)
            path = config_dir / "config_paths.yaml"
            path.write_text(
                "app:\n"
                "  robot_controller: ''\n"
                "robot:\n"
                "  can_device: qhrr0/config/robot_config/can_device_config.yaml\n"
                "policy:\n"
                "  policy_list: qhrr0/app/robot_controller/subprocesses/"
                "task_controller/policy_config/qhrr/policy_list.yaml\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(AppConfigError, "robot_controller"):
                load_config_paths(path)


if __name__ == "__main__":
    unittest.main()
