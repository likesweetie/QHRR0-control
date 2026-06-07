# app/robot_controller/main.py
from __future__ import annotations

import argparse
import signal
from pathlib import Path

from ...factory.app_factory.app_factory import (
    AppFactory,
    HardwareSafetyOptions,
)
from ...qhrr0 import QHRR0

from .config import load_robot_controller_config
from .controller import RobotController


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "config"
    / "app_config"
    / "robot_controller.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR0 RobotController runtime")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="RobotController YAML config path",
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Explicitly request hardware mode when runtime.mode is hardware.",
    )
    parser.add_argument(
        "--i-understand-this-can-enable-motors",
        action="store_true",
        help="Confirm this run may enable real motors in hardware mode.",
    )
    parser.add_argument(
        "--estop-ok",
        action="store_true",
        help="Declare that the hardware E-stop path was checked before startup.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = load_robot_controller_config(args.config)

    safety_options = HardwareSafetyOptions(
        hardware_requested=bool(args.hardware),
        motor_enable_confirmed=bool(args.i_understand_this_can_enable_motors),
        estop_ok=bool(args.estop_ok),
    )

    # Factory stage:
    # - validates app config
    # - validates robot/config consistency
    # - creates implementation-agnostic build spec
    #
    # Current RobotController implementation still consumes config directly,
    # so the returned spec is intentionally not used for runtime construction yet.
    _app_spec = AppFactory().create_app_spec(
        robot=QHRR0,
        config=config,
        options=safety_options,
    )

    controller = RobotController(config)

    def handle_signal(signum: int, _frame: object) -> None:
        print(f"[robot_controller] signal {signum}, shutting down")
        controller.request_stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        controller.start()
        controller.run()
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()