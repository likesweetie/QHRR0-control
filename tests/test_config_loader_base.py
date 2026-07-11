from pathlib import Path
from types import MappingProxyType
from typing import Any

import pytest

from qhrr0.app.helper.config_manage import (
    ConfigLoaderBase,
    FrozenJsonArray,
    FrozenJsonObject,
    YAMLconfigLoader,
)


def test_load_yaml_as_read_only_runtime_snapshot(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        "\n".join(
            [
                "robot:",
                "  name: qhrr",
                "  actuators:",
                "    - can_id: 321",
                "      enabled: true",
            ]
        ),
        encoding="utf-8",
    )

    snapshot = ConfigLoaderBase().load(config_path)

    assert snapshot.source_path == config_path
    assert snapshot.require_path("robot.name", str) == "qhrr"
    assert snapshot.require_path(("robot", "actuators"), FrozenJsonArray)
    assert snapshot.require_path(("robot", "actuators", 0, "can_id"), int) == 321

    root = snapshot.root
    assert isinstance(root, FrozenJsonObject)
    with pytest.raises(TypeError):
        root["robot"] = "mutated"


def test_to_json_returns_mutable_copy_without_mutating_snapshot(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text("robot:\n  name: qhrr\n", encoding="utf-8")

    snapshot = ConfigLoaderBase().load(config_path)
    exported = snapshot.to_json()
    exported["robot"]["name"] = "changed"

    assert snapshot.require_path("robot.name") == "qhrr"


def test_missing_path_and_type_mismatch_are_not_silent(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text("robot:\n  name: qhrr\n", encoding="utf-8")

    snapshot = ConfigLoaderBase().load(config_path)

    with pytest.raises(KeyError):
        snapshot.at("robot.missing")
    with pytest.raises(TypeError):
        snapshot.require_path("robot.name", int)


def test_yaml_root_must_be_object(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text("- not\n- object\n", encoding="utf-8")

    with pytest.raises(ValueError, match="root must be a mapping"):
        ConfigLoaderBase().load(config_path)


def test_non_json_yaml_value_is_rejected(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text("created_at: 2026-06-11\n", encoding="utf-8")

    with pytest.raises(TypeError, match="Unsupported JSON value type"):
        ConfigLoaderBase().load(config_path)


def test_yaml_config_loader_returns_immutable_object(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        "\n".join(
            [
                "robot:",
                "  name: qhrr",
                "  actuators:",
                "    - can_id: 321",
            ]
        ),
        encoding="utf-8",
    )

    config = YAMLconfigLoader(config_path).load()

    assert isinstance(config, MappingProxyType)
    assert config["robot"]["name"] == "qhrr"
    assert isinstance(config["robot"], MappingProxyType)
    assert isinstance(config["robot"]["actuators"], tuple)
    assert isinstance(config["robot"]["actuators"][0], MappingProxyType)
    with pytest.raises(TypeError):
        config["robot"] = {}
    with pytest.raises(TypeError):
        config["robot"]["name"] = "changed"
    with pytest.raises(TypeError):
        config["robot"]["actuators"][0]["can_id"] = 0


def test_yaml_config_loader_runs_optional_validator(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text("robot:\n  name: qhrr\n", encoding="utf-8")
    seen = []

    def validator(config: dict[str, Any]) -> None:
        seen.append(config["robot"]["name"])

    config = YAMLconfigLoader(config_path, validator=validator).load()

    assert config["robot"]["name"] == "qhrr"
    assert seen == ["qhrr"]
