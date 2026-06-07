# qhrr0/main.py
from __future__ import annotations

import argparse
import signal
from pathlib import Path

from qhrr0.app.robot_controller.config import load_robot_controller_config
from qhrr0.app.robot_controller.controller import RobotController
from qhrr0.factory.app_factory.app_factory import AppFactory, HardwareSafetyOptions
from qhrr0.qhrr0 import QHRR0


DEFAULT_CONFIG = (
    Path(__file__).resolve().parent
    / "config"
    / "app_config"
    / "robot_controller.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR0 runtime")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="QHRR0 robot controller YAML config path",
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

    # Build/validation stage.
    # AppFactory must not create app runtime implementation objects.
    # It validates config + QHRR0 consistency and emits AppBuildSpec.
    _app_spec = AppFactory().create_app_spec(
        robot=QHRR0,
        config=config,
        options=safety_options,
    )

    # Legacy runtime stage.
    # Current RobotController still consumes config directly.
    # Do not rewrite it in this pass.
    controller = RobotController(config)

    def handle_signal(signum: int, _frame: object) -> None:
        print(f"[qhrr0] signal {signum}, shutting down")
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