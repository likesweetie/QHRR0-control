from __future__ import annotations

from pathlib import Path

from robot_controller.config.paths import ConfigPathRegistry, load_config_paths


def resolve_config_arg(
    path: str | Path | None,
    key: str | None,
    *,
    default_key: str,
    config_paths: ConfigPathRegistry | None = None,
) -> Path:
    if path is not None:
        return Path(path).resolve()
    paths = load_config_paths() if config_paths is None else config_paths
    return paths.config(key or default_key)
