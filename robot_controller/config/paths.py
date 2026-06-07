from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from robot_controller.config.loader import (
    ConfigError,
    load_yaml_mapping,
    require_mapping,
)


DEFAULT_CONFIG_PATHS = Path("config/app_config/config_paths.yaml")


@dataclass(frozen=True)
class ConfigPathRegistry:
    configs: Mapping[str, Path]
    policy: Mapping[str, Path]

    def config(self, key: str) -> Path:
        try:
            return self.configs[key]
        except KeyError as exc:
            raise ConfigError(f"unknown config path key: configs.{key}") from exc

    def policy_path(self, key: str) -> Path:
        try:
            return self.policy[key]
        except KeyError as exc:
            raise ConfigError(f"unknown config path key: policy.{key}") from exc


def load_config_paths(path: str | Path = DEFAULT_CONFIG_PATHS) -> ConfigPathRegistry:
    config_path = Path(path)
    raw = load_yaml_mapping(config_path)
    base = _project_root_from_config_paths(config_path)

    return ConfigPathRegistry(
        configs=_parse_path_mapping(require_mapping(raw, "configs", str(config_path)), base),
        policy=_parse_path_mapping(require_mapping(raw, "policy", str(config_path)), base),
    )


def _project_root_from_config_paths(path: Path) -> Path:
    resolved = path.resolve()
    try:
        return resolved.parents[2]
    except IndexError as exc:
        raise ConfigError(f"config paths file is too shallow to resolve project root: {path}") from exc


def _parse_path_mapping(raw: Mapping[object, object], base: Path) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for key, value in raw.items():
        if key is None:
            raise ConfigError("config path keys must not be empty")
        if value is None:
            raise ConfigError(f"config path value must not be empty: {key}")
        key_text = str(key)
        value_text = str(value)
        if not key_text:
            raise ConfigError("config path keys must not be empty")
        if not value_text:
            raise ConfigError(f"config path value must not be empty: {key_text}")
        candidate = Path(value_text)
        if not candidate.is_absolute():
            candidate = base / candidate
        paths[key_text] = candidate.resolve()
    return paths
