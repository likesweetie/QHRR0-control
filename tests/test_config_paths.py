from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from robot_controller.config import ConfigError, load_config_paths


class ConfigPathRegistryTest(unittest.TestCase):
    def test_default_config_paths_resolve_from_project_root(self) -> None:
        paths = load_config_paths()

        self.assertEqual(
            paths.config("robot_controller"),
            Path("config/app_config/robot_controller.yaml").resolve(),
        )
        self.assertEqual(
            paths.config("robot_platform"),
            Path("config/app_config/robot_platform.yaml").resolve(),
        )
        self.assertEqual(
            paths.policy_path("policy_list"),
            Path("config/policy_config/qhrr/policy_list.yaml").resolve(),
        )
        self.assertTrue(paths.config("policy_runner").exists())

    def test_unknown_keys_raise_config_error(self) -> None:
        paths = load_config_paths()

        with self.assertRaisesRegex(ConfigError, "configs.missing"):
            paths.config("missing")
        with self.assertRaisesRegex(ConfigError, "policy.missing"):
            paths.policy_path("missing")

    def test_empty_path_values_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config_dir = root / "config" / "app_config"
            config_dir.mkdir(parents=True)
            path = config_dir / "config_paths.yaml"
            path.write_text(
                "configs:\n"
                "  robot_controller: ''\n"
                "policy:\n"
                "  policy_list: config/policy_config/qhrr/policy_list.yaml\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ConfigError, "robot_controller"):
                load_config_paths(path)


if __name__ == "__main__":
    unittest.main()
