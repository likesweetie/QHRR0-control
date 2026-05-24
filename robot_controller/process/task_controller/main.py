from __future__ import annotations

import argparse
import os
import signal
import time
from pathlib import Path

from robot_controller.core.config import load_robot_controller_config
from robot_controller.core.state import MitTarget
from robot_controller.process.task_controller.policy import (
    action_offset,
    load_policies,
    load_yaml,
    project_root,
    resolve_policy_config_dir,
)
from robot_controller.process.task_controller.shm_io import (
    AuxCommandReader,
    ControlStateReader,
    state_vectors,
)
from robot_controller.utils.shm_command_router import ShmMitCommandWriter


RUNNING = True


def _handle_signal(signum: int, _frame) -> None:
    global RUNNING
    print(f"[task_controller] signal {signum}, shutting down", flush=True)
    RUNNING = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR Python task controller")
    parser.add_argument("--controller-config", default=os.environ.get("ROBOT_CONTROLLER_CONFIG", "app_config/robot_controller.yaml"))
    parser.add_argument("--robot-name", default=os.environ.get("ROBOT_NAME"))
    parser.add_argument("--project-root", default=os.environ.get("QHRR_PROJECT_ROOT", "."))
    parser.add_argument("--policy-config-dir", default=os.environ.get("POLICY_CONFIG_DIR"))
    parser.add_argument("--control-hz", type=float, default=float(os.environ.get("TASK_CONTROL_HZ", "50.0")))
    parser.add_argument("--state-timeout-s", type=float, default=float(os.environ.get("TASK_STATE_TIMEOUT_S", "0.25")))
    parser.add_argument("--startup-timeout-s", type=float, default=float(os.environ.get("TASK_STARTUP_TIMEOUT_S", "5.0")))
    parser.add_argument("--aux-timeout-s", type=float, default=float(os.environ.get("TASK_AUX_TIMEOUT_S", "0.5")))
    return parser.parse_args()


def _should_run() -> bool:
    return RUNNING


def _controller_config_path(root: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else root / path


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    root = project_root(args.project_root)
    controller_config = load_robot_controller_config(_controller_config_path(root, args.controller_config))
    robot_name = args.robot_name or controller_config.platform.robot.name
    robot_assets = controller_config.platform.robots[robot_name]
    policy_config_dir = resolve_policy_config_dir(
        root,
        args.policy_config_dir or robot_assets.policy_config_dir,
        robot_name,
    )
    active_policy = load_policies(root, policy_config_dir)[0]
    pd_config = load_yaml(active_policy.directory / "pd_config.yaml")
    kp = float(pd_config["kp"])
    kd = float(pd_config["kd"])

    can_ids = [int(can_id) for can_id in controller_config.can.motors.can_ids]

    control_reader = ControlStateReader(controller_config.shm.control_state.name)
    aux_reader = AuxCommandReader(controller_config.shm.aux_command.name)
    mit_writer = ShmMitCommandWriter(controller_config.shm.mit_command)
    print(
        f"[task_controller] control={controller_config.shm.control_state.name} "
        f"aux={controller_config.shm.aux_command.name} mit={controller_config.shm.mit_command.name}",
        flush=True,
    )

    period_s = 1.0 / args.control_hz
    try:
        control_reader.wait_until_available(
            read_timeout_s=args.state_timeout_s,
            startup_timeout_s=args.startup_timeout_s,
            should_run=_should_run,
        )

        while RUNNING:
            tick_start = time.monotonic()

            control_state = control_reader.latest(args.state_timeout_s)
            lin_vel, ang_vel_cmd, buttons = aux_reader.latest(args.aux_timeout_s)
            dof_pos, dof_vel, quat, gyro = state_vectors(control_state, can_ids)

            mode = bool(buttons.get("a_button", False))
            active_policy.set_state(dof_pos, dof_vel, quat, gyro)
            active_policy.set_commands(float(lin_vel[0]), float(lin_vel[1]), float(ang_vel_cmd[2]), mode)
            q_target = active_policy.compute_action() + action_offset(active_policy, robot_name, dof_pos, mode)

            mit_writer.publish(
                [
                    MitTarget(
                        can_id=can_id,
                        position_rad=float(q_target[index]),
                        velocity_rad_s=0.0,
                        kp=kp,
                        kd=kd,
                        torque_ff_nm=0.0,
                    )
                    for index, can_id in enumerate(can_ids)
                ]
            )

            elapsed_s = time.monotonic() - tick_start
            if elapsed_s < period_s:
                time.sleep(period_s - elapsed_s)
            else:
                print(f"[task_controller] loop overrun: {elapsed_s:.6f}s", flush=True)
    finally:
        control_reader.close()
        aux_reader.close()
        mit_writer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
