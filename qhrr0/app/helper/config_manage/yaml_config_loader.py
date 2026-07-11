from __future__ import annotations

from collections.abc import Callable, Mapping
from pathlib import Path
from types import MappingProxyType
from typing import Any

from .base.config_loader_base import ConfigLoaderBase


ImmutableJsonObject = Mapping[str, Any]
ConfigValidator = Callable[[ImmutableJsonObject], None]


class YAMLconfigLoader:
    """Simple YAML config loader interface.

    It loads one YAML file and returns an immutable JSON object.
    Validation is intentionally optional until the validator contract is fixed.
    """

    def __init__(
        self,
        config_path: Path | str,
        validator: ConfigValidator | None = None,
        *,
        base_dir: Path | str | None = None,
    ) -> None:
        self.config_path = Path(config_path)
        self.validator = validator
        self._base_loader = ConfigLoaderBase(base_dir=base_dir)

    def load(self) -> ImmutableJsonObject:
        snapshot = self._base_loader.load(self.config_path)
        config = _freeze_json_object(snapshot.to_json())
        if self.validator is not None:
            self.validator(config)
        return config


def _freeze_json_object(value: Mapping[str, Any]) -> ImmutableJsonObject:
    return MappingProxyType(
        {
            key: _freeze_json_value(item)
            for key, item in value.items()
        }
    )


def _freeze_json_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return _freeze_json_object(value)
    if isinstance(value, list | tuple):
        return tuple(_freeze_json_value(item) for item in value)
    if isinstance(value, str | int | float | bool) or value is None:
        return value
    raise TypeError(f"Unsupported JSON value type: {type(value).__name__}")
