from __future__ import annotations

import argparse
import errno
import os
import signal
import struct
import time
from pathlib import Path

import yaml

from qhrr0.app.robot_controller.shm import AuxCommandC, AuxCommandShm
from qhrr0.app.robot_controller.subprocesses.joystick_reader.joystick_buttons import buttons_to_mask


PACKAGE_ROOT = Path(__file__).resolve().parents[4]
PROJECT_ROOT = Path(__file__).resolve().parents[5]
CONFIG_PATHS_PATH = PACKAGE_ROOT / "config" / "config_paths.yaml"


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_STRUCT = struct.Struct("IhBB")


RUNNING = True


def _handle_signal(signum: int, _frame) -> None:
    global RUNNING
    print(f"[joystick_reader] signal {signum}, shutting down", flush=True)
    RUNNING = False


def _axis_value(value: int) -> float:
    return max(-1.0, min(1.0, float(value) / 32767.0))


def _deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def _axis_targets(axes: list[float]) -> tuple[list[float], list[float]]:
    mappings = (
        (1, 0, 1.0, True, 0.05, "lin"),
        (0, 1, 1.0, True, 0.05, "lin"),
        (3, 2, 1.0, True, 0.05, "ang"),
    )
    lin_vel_target = [0.0, 0.0, 0.0]
    ang_vel_target = [0.0, 0.0, 0.0]
    for axis, index, scale, invert, deadband, target in mappings:
        value = _deadband(axes[axis], deadband)
        if invert:
            value = -value
        if target == "lin":
            lin_vel_target[index] = value * scale
        else:
            ang_vel_target[index] = value * scale
    return lin_vel_target, ang_vel_target


def _button_targets(buttons: list[bool]) -> dict[str, bool]:
    fields = (
        "a_button",
        "b_button",
        "x_button",
        "y_button",
        "lb_button",
        "rb_button",
        "back_button",
        "start_button",
        "guide_button",
        "l3_button",
        "r3_button",
    )
    return {field: bool(buttons[index]) for index, field in enumerate(fields)}


def _publish(writer: AuxCommandShm, axes: list[float], buttons: list[bool]) -> None:
    lin_vel_target, ang_vel_target = _axis_targets(axes)
    command = AuxCommandC()
    command.timestamp_ns = time.time_ns()
    for index in range(3):
        command.lin_vel_target[index] = float(lin_vel_target[index])
        command.ang_vel_target[index] = float(ang_vel_target[index])
    command.button_mask = buttons_to_mask(_button_targets(buttons))
    writer.write(command)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR joystick reader")
    parser.add_argument("--controller-config", type=Path, default=None)
    parser.add_argument("--controller-config-key", default=os.environ.get("ROBOT_CONTROLLER_CONFIG_KEY", "robot_controller"))
    parser.add_argument("--joystick-dev", default=os.environ.get("JOYSTICK_DEV", "/dev/input/js0"))
    parser.add_argument("--poll-sleep-s", type=float, default=0.001)
    return parser.parse_args()


def _load_yaml_mapping(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fp:
        loaded = yaml.safe_load(fp)
    if not isinstance(loaded, dict):
        raise ValueError(f"YAML file must contain a mapping: {path}")
    return loaded


def _resolve_controller_config_path(explicit_path: Path | None, key: str) -> Path:
    if explicit_path is not None:
        return explicit_path if explicit_path.is_absolute() else PROJECT_ROOT / explicit_path

    config_paths = _load_yaml_mapping(CONFIG_PATHS_PATH)
    app_paths = config_paths["app"]
    if not isinstance(app_paths, dict):
        raise TypeError("config_paths.app must be a mapping")
    config_path = Path(str(app_paths[key]))
    return config_path if config_path.is_absolute() else PACKAGE_ROOT / config_path


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    config_path = _resolve_controller_config_path(args.controller_config, args.controller_config_key)
    config = _load_yaml_mapping(config_path)
    aux_shm_name = str(config["shm"]["aux_command"]["name"])
    writer = AuxCommandShm.open(aux_shm_name)
    print(f"[joystick_reader] publishing aux command shm: {aux_shm_name}", flush=True)

    fd = os.open(args.joystick_dev, os.O_RDONLY | os.O_NONBLOCK)
    print(f"[joystick_reader] joystick device: {args.joystick_dev}", flush=True)

    axes = [0.0] * 32
    buttons = [False] * 32
    try:
        _publish(writer, axes, buttons)
        print("[joystick_reader] published neutral aux command", flush=True)

        while RUNNING:
            try:
                packet = os.read(fd, JS_EVENT_STRUCT.size)
            except BlockingIOError:
                time.sleep(args.poll_sleep_s)
                continue
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    time.sleep(args.poll_sleep_s)
                    continue
                raise

            if len(packet) != JS_EVENT_STRUCT.size:
                raise RuntimeError(f"partial joystick event read: {len(packet)} bytes")

            _timestamp_ms, value, event_type, number = JS_EVENT_STRUCT.unpack(packet)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS and number < len(axes):
                axes[number] = _axis_value(value)
                _publish(writer, axes, buttons)
            elif event_type == JS_EVENT_BUTTON and number < len(buttons):
                buttons[number] = value != 0
                _publish(writer, axes, buttons)
    finally:
        os.close(fd)
        writer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
