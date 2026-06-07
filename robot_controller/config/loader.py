from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    pass


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as fp:
        raw = yaml.safe_load(fp)
    if not isinstance(raw, dict):
        raise ConfigError(f"{config_path} must contain a YAML mapping")
    return raw


def resolve_config_path(base_path: str | Path, value: str, path: str) -> Path:
    if not value:
        raise ConfigError(f"Config key must not be empty: {path}")
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = Path(base_path).parent / candidate
    return candidate.resolve()


def require_key(mapping: dict[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise ConfigError(f"Missing required config key: {path}.{key}")
    return mapping[key]


def require_mapping(mapping: dict[str, Any], key: str, path: str) -> dict[str, Any]:
    value = require_key(mapping, key, path)
    if not isinstance(value, dict):
        raise ConfigError(f"Config key must be a mapping: {path}.{key}")
    return value


def require_list(mapping: dict[str, Any], key: str, path: str) -> list[Any]:
    value = require_key(mapping, key, path)
    if not isinstance(value, list):
        raise ConfigError(f"Config key must be a list: {path}.{key}")
    return value


def require_bool(mapping: dict[str, Any], key: str, path: str) -> bool:
    value = require_key(mapping, key, path)
    if not isinstance(value, bool):
        raise ConfigError(f"Config key must be a boolean: {path}.{key}")
    return value


def require_float(mapping: dict[str, Any], key: str, path: str) -> float:
    return float(require_key(mapping, key, path))


def require_int(mapping: dict[str, Any], key: str, path: str) -> int:
    return int(require_key(mapping, key, path))


def optional_float_or_none(mapping: dict[str, Any], key: str, path: str) -> float | None:
    value = require_key(mapping, key, path)
    return None if value is None else float(value)


def parse_int(value: Any) -> int:
    return int(value, 0) if isinstance(value, str) else int(value)
