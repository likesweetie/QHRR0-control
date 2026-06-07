from __future__ import annotations

import argparse
import signal
from pathlib import Path

from qhrr0.app.robot_controller.controller import RobotController
from qhrr0.app.robot_controller.process_supervisor import ProcessSupervisor
from qhrr0.factory.app_factory.app_factory import (
    DEFAULT_CONTROLLER_CONFIG,
    AppFactory,
)
from qhrr0.factory.app_factory.schema import ProcessLaunchSpec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR0 runtime")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONTROLLER_CONFIG,
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


def build_process_supervisor(processes: tuple[ProcessLaunchSpec, ...]) -> ProcessSupervisor:
    supervisor = ProcessSupervisor()
    for process in processes:
        supervisor.add_process(
            name=process.name,
            command=process.command,
            start_order=process.start_order,
            stop_order=process.stop_order,
            new_terminal=process.new_terminal,
            terminal_command=process.terminal_command,
            working_dir=process.working_dir,
            env_vars=dict(process.env_vars),
        )
    return supervisor


def main() -> None:
    args = parse_args()
    runtime_spec = AppFactory().create_robot_controller_runtime_spec(
        controller_config_path=args.config,
        hardware_requested=bool(args.hardware),
        motor_enable_confirmed=bool(args.i_understand_this_can_enable_motors),
        estop_ok=bool(args.estop_ok),
    )

    process_supervisor = build_process_supervisor(runtime_spec.processes)
    controller = RobotController(
        runtime=runtime_spec.controller,
        process_supervisor=process_supervisor,
    )

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
