from __future__ import annotations

import argparse
import os
from pathlib import Path

from robot_controller.config import resolve_config_arg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR dashboard backend")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--config-key", default="dashboard")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = resolve_config_arg(args.config, args.config_key, default_key="dashboard")
    os.environ["DASHBOARD_CONFIG"] = str(config_path)

    from robot_controller.subprocesses.dashboard.backend.app import main as backend_main

    backend_main()


if __name__ == "__main__":
    main()
