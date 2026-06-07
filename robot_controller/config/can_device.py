from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_controller.config.loader import (
    ConfigError,
    load_yaml_mapping,
    parse_int,
    require_bool,
    require_key,
    require_mapping,
)


class CanDeviceConfigError(ConfigError):
    pass


@dataclass(frozen=True)
class CanDeviceImuConfig:
    type: str
    request_id: int
    quat_id: int
    gyro_id: int
    cmd_get_quat: int
    cmd_get_gyro: int
    cmd_get_all: int
    quat_scale: float
    gyro_scale: float
    normalize_quat: bool


@dataclass(frozen=True)
class SpgMitDriverConfig:
    p_max_rad: float
    v_max_rad_s: float
    kp_max: float
    kd_max: float
    tau_max_nm: float
    feedback_position_max_rad: float
    iq_full_scale_count: float
    iq_full_scale_current_a: float
    set_zero_hold_s: float


@dataclass(frozen=True)
class CanDeviceConfig:
    path: Path
    imu: CanDeviceImuConfig
    drivers: dict[str, SpgMitDriverConfig]


def load_can_device_config(path: str | Path) -> CanDeviceConfig:
    config_path = Path(path).resolve()
    raw = load_yaml_mapping(config_path)
    config = CanDeviceConfig(
        path=config_path,
        imu=parse_imu_config(require_mapping(raw, "imu", "<root>")),
        drivers=parse_driver_configs(require_mapping(raw, "drivers", "<root>")),
    )
    validate_can_device_config(config)
    return config


def parse_imu_config(raw: dict[str, Any]) -> CanDeviceImuConfig:
    config = CanDeviceImuConfig(
        type=_require_non_empty_string(raw, "type", "imu"),
        request_id=parse_int(require_key(raw, "request_id", "imu")),
        quat_id=parse_int(require_key(raw, "quat_id", "imu")),
        gyro_id=parse_int(require_key(raw, "gyro_id", "imu")),
        cmd_get_quat=parse_int(require_key(raw, "cmd_get_quat", "imu")),
        cmd_get_gyro=parse_int(require_key(raw, "cmd_get_gyro", "imu")),
        cmd_get_all=parse_int(require_key(raw, "cmd_get_all", "imu")),
        quat_scale=float(require_key(raw, "quat_scale", "imu")),
        gyro_scale=float(require_key(raw, "gyro_scale", "imu")),
        normalize_quat=require_bool(raw, "normalize_quat", "imu"),
    )
    validate_imu_config(config)
    return config


def parse_driver_configs(raw: dict[str, Any]) -> dict[str, SpgMitDriverConfig]:
    drivers: dict[str, SpgMitDriverConfig] = {}
    for key, value in raw.items():
        name = _non_empty_string(key, "drivers key")
        if not isinstance(value, dict):
            raise CanDeviceConfigError(f"Config key must be a mapping: drivers.{name}")
        drivers[name] = parse_spg_mit_driver_config(value, f"drivers.{name}")
    if not drivers:
        raise CanDeviceConfigError("drivers must not be empty")
    return drivers


def parse_spg_mit_driver_config(raw: dict[str, Any], path: str) -> SpgMitDriverConfig:
    config = SpgMitDriverConfig(
        p_max_rad=float(require_key(raw, "p_max_rad", path)),
        v_max_rad_s=float(require_key(raw, "v_max_rad_s", path)),
        kp_max=float(require_key(raw, "kp_max", path)),
        kd_max=float(require_key(raw, "kd_max", path)),
        tau_max_nm=float(require_key(raw, "tau_max_nm", path)),
        feedback_position_max_rad=float(require_key(raw, "feedback_position_max_rad", path)),
        iq_full_scale_count=float(require_key(raw, "iq_full_scale_count", path)),
        iq_full_scale_current_a=float(require_key(raw, "iq_full_scale_current_a", path)),
        set_zero_hold_s=float(require_key(raw, "set_zero_hold_s", path)),
    )
    validate_spg_mit_driver_config(config, path)
    return config


def validate_can_device_config(config: CanDeviceConfig) -> None:
    validate_imu_config(config.imu)
    for name, driver in config.drivers.items():
        _non_empty_string(name, "drivers key")
        validate_spg_mit_driver_config(driver, f"drivers.{name}")


def validate_imu_config(config: CanDeviceImuConfig) -> None:
    if config.type != "e2box":
        raise CanDeviceConfigError("imu.type must be 'e2box'")
    if config.request_id <= 0:
        raise CanDeviceConfigError("imu.request_id must be > 0")
    if config.quat_id <= 0:
        raise CanDeviceConfigError("imu.quat_id must be > 0")
    if config.gyro_id <= 0:
        raise CanDeviceConfigError("imu.gyro_id must be > 0")
    if config.cmd_get_quat < 0:
        raise CanDeviceConfigError("imu.cmd_get_quat must be >= 0")
    if config.cmd_get_gyro < 0:
        raise CanDeviceConfigError("imu.cmd_get_gyro must be >= 0")
    if config.cmd_get_all < 0:
        raise CanDeviceConfigError("imu.cmd_get_all must be >= 0")
    if config.quat_scale <= 0.0:
        raise CanDeviceConfigError("imu.quat_scale must be > 0")
    if config.gyro_scale <= 0.0:
        raise CanDeviceConfigError("imu.gyro_scale must be > 0")


def validate_spg_mit_driver_config(config: SpgMitDriverConfig, path: str) -> None:
    if config.p_max_rad <= 0.0:
        raise CanDeviceConfigError(f"{path}.p_max_rad must be > 0")
    if config.v_max_rad_s <= 0.0:
        raise CanDeviceConfigError(f"{path}.v_max_rad_s must be > 0")
    if config.kp_max <= 0.0:
        raise CanDeviceConfigError(f"{path}.kp_max must be > 0")
    if config.kd_max <= 0.0:
        raise CanDeviceConfigError(f"{path}.kd_max must be > 0")
    if config.tau_max_nm <= 0.0:
        raise CanDeviceConfigError(f"{path}.tau_max_nm must be > 0")
    if config.feedback_position_max_rad <= 0.0:
        raise CanDeviceConfigError(f"{path}.feedback_position_max_rad must be > 0")
    if config.iq_full_scale_count <= 0.0:
        raise CanDeviceConfigError(f"{path}.iq_full_scale_count must be > 0")
    if config.iq_full_scale_current_a <= 0.0:
        raise CanDeviceConfigError(f"{path}.iq_full_scale_current_a must be > 0")
    if config.set_zero_hold_s < 0.0:
        raise CanDeviceConfigError(f"{path}.set_zero_hold_s must be >= 0")


def _require_non_empty_string(raw: dict[str, Any], key: str, path: str) -> str:
    return _non_empty_string(require_key(raw, key, path), f"{path}.{key}")


def _non_empty_string(value: Any, path: str) -> str:
    if value is None:
        raise CanDeviceConfigError(f"Config key must not be empty: {path}")
    text = str(value).strip()
    if not text:
        raise CanDeviceConfigError(f"Config key must not be empty: {path}")
    return text
