from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from pathlib import Path
from typing import Any, TypeAlias

import yaml


JsonScalar: TypeAlias = str | int | float | bool | None
JsonPathPart: TypeAlias = str | int
JsonPath: TypeAlias = str | Sequence[JsonPathPart]


class _Missing:
    pass


MISSING = _Missing()


class FrozenJsonArray(Sequence[Any]):
    """Read-only wrapper for a JSON array."""

    __slots__ = ("_items",)

    def __init__(self, items: Sequence[Any]) -> None:
        self._items = tuple(_freeze_json_value(item) for item in items)

    def __getitem__(self, index):
        return self._items[index]

    def __iter__(self) -> Iterator[Any]:
        return iter(self._items)

    def __len__(self) -> int:
        return len(self._items)

    def to_json(self) -> list[Any]:
        return [_thaw_json_value(item) for item in self._items]


class FrozenJsonObject(Mapping[str, Any]):
    """Read-only wrapper for a JSON object."""

    __slots__ = ("_data",)

    def __init__(self, data: Mapping[str, Any]) -> None:
        frozen: dict[str, Any] = {}
        for key, value in data.items():
            if not isinstance(key, str):
                raise TypeError(f"JSON object key must be str, got {type(key).__name__}")
            frozen[key] = _freeze_json_value(value)
        self._data = frozen

    def __getitem__(self, key: str) -> Any:
        return self._data[key]

    def __iter__(self) -> Iterator[str]:
        return iter(self._data)

    def __len__(self) -> int:
        return len(self._data)

    def at(self, path: JsonPath, default: Any = MISSING) -> Any:
        parts = _normalize_path(path)
        current: Any = self
        for part in parts:
            try:
                current = _read_path_part(current, part)
            except (KeyError, IndexError, TypeError):
                if default is not MISSING:
                    return default
                raise
        return current

    def require_path(self, path: JsonPath, expected_type: type | tuple[type, ...] | None = None) -> Any:
        value = self.at(path)
        if expected_type is not None and not isinstance(value, expected_type):
            path_text = ".".join(str(part) for part in _normalize_path(path))
            raise TypeError(
                f"Config path '{path_text}' must be {expected_type}, "
                f"got {type(value).__name__}"
            )
        return value

    def to_json(self) -> dict[str, Any]:
        return {
            key: _thaw_json_value(value)
            for key, value in self._data.items()
        }


class RuntimeConfigSnapshot:
    """Immutable runtime snapshot created from a YAML config file."""

    __slots__ = ("source_path", "_root")

    def __init__(self, *, source_path: Path, root: Mapping[str, Any]) -> None:
        self.source_path = source_path
        self._root = FrozenJsonObject(root)

    @property
    def root(self) -> FrozenJsonObject:
        return self._root

    def at(self, path: JsonPath, default: Any = MISSING) -> Any:
        return self._root.at(path, default=default)

    def require_path(self, path: JsonPath, expected_type: type | tuple[type, ...] | None = None) -> Any:
        return self._root.require_path(path, expected_type=expected_type)

    def to_json(self) -> dict[str, Any]:
        return self._root.to_json()


class ConfigLoaderBase:
    """Base YAML loader that returns a read-only runtime JSON snapshot."""

    def __init__(self, *, base_dir: Path | str | None = None) -> None:
        self.base_dir = Path(base_dir).resolve() if base_dir is not None else None

    def load(self, path: Path | str) -> RuntimeConfigSnapshot:
        source_path = self.resolve_path(path)
        raw = self.load_yaml_object(source_path)
        prepared = self.prepare(raw, source_path=source_path)
        snapshot = RuntimeConfigSnapshot(source_path=source_path, root=prepared)
        self.validate(snapshot)
        return snapshot

    def resolve_path(self, path: Path | str) -> Path:
        resolved = Path(path)
        if not resolved.is_absolute() and self.base_dir is not None:
            resolved = self.base_dir / resolved
        resolved = resolved.resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"Config file not found: {resolved}")
        return resolved

    def load_yaml_object(self, path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as fp:
            loaded = yaml.safe_load(fp)
        if not isinstance(loaded, dict):
            raise ValueError(f"YAML config root must be a mapping: {path}")
        return loaded

    def prepare(self, raw: Mapping[str, Any], *, source_path: Path) -> Mapping[str, Any]:
        return raw

    def validate(self, snapshot: RuntimeConfigSnapshot) -> None:
        return None


def _freeze_json_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return FrozenJsonObject(value)
    if isinstance(value, list | tuple):
        return FrozenJsonArray(value)
    if isinstance(value, str | int | float | bool) or value is None:
        return value
    raise TypeError(f"Unsupported JSON value type: {type(value).__name__}")


def _thaw_json_value(value: Any) -> Any:
    if isinstance(value, FrozenJsonObject | FrozenJsonArray):
        return value.to_json()
    return value


def _normalize_path(path: JsonPath) -> tuple[JsonPathPart, ...]:
    if isinstance(path, str):
        if not path:
            raise ValueError("Config path must not be empty")
        return tuple(path.split("."))
    parts = tuple(path)
    if not parts:
        raise ValueError("Config path must not be empty")
    return parts


def _read_path_part(current: Any, part: JsonPathPart) -> Any:
    if isinstance(current, FrozenJsonObject):
        if not isinstance(part, str):
            raise TypeError(f"JSON object path part must be str, got {type(part).__name__}")
        return current[part]
    if isinstance(current, FrozenJsonArray):
        if not isinstance(part, int):
            raise TypeError(f"JSON array path part must be int, got {type(part).__name__}")
        return current[part]
    raise TypeError(f"Cannot read path part from {type(current).__name__}")
