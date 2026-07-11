# app/robot_controller/main.py
from __future__ import annotations

import argparse
import signal
from pathlib import Path

from ...factory.app_factory.app_factory import (
    AppFactory,
    HardwareSafetyOptions,
)
from ...qhrr0_spec import QHRR0

from .config import load_robot_controller_config
from .controller import RobotController


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "config"
    / "app_config"
    / "robot_controller.yaml"
)




def main() -> None:
    
    controller = RobotController()

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