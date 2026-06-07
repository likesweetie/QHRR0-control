from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR dashboard subprocess")
    parser.add_argument("--runtime-config", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    from qhrr0.app.robot_controller.subprocesses.dashboard.backend.app import main as backend_main

    args = parse_args()
    backend_main(args.runtime_config)


if __name__ == "__main__":
    main()
