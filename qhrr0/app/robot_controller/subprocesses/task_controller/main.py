from __future__ import annotations

import argparse
import signal
import time
from pathlib import Path

import numpy as np

from qhrr0.app.robot_controller.subprocesses.task_controller.policy_runner import (
    action_offset,
    load_policies,
    load_yaml,
    project_root,
    resolve_policy_config_dir,
)
from qhrr0.app.robot_controller.shm.types.aux_command import AuxCommandShm
from qhrr0.app.robot_controller.shm.types.control_command import (
    MAX_CONTROL_TARGETS,
    ControlCommandC,
    ControlCommandShm,
)
from qhrr0.app.robot_controller.shm.types.robot_state import RobotStateShm
from qhrr0.app.robot_controller.subprocesses.aux_buttons import mask_to_buttons


RUNNING = True


def _handle_signal(signum: int, _frame) -> None:
    global RUNNING
    print(f"[task_controller] signal {signum}, shutting down", flush=True)
    RUNNING = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR Python task controller")
    parser.add_argument("--policy-runner-config", type=Path, required=True)
    parser.add_argument("--policy-list", type=Path, required=True)
    parser.add_argument("--pd-config", type=Path, required=True)
    parser.add_argument("--robot-name", required=True)
    parser.add_argument("--can-ids", required=True)
    parser.add_argument("--control-state-shm-name", required=True)
    parser.add_argument("--aux-command-shm-name", required=True)
    parser.add_argument("--mit-command-shm-name", required=True)
    parser.add_argument("--project-root", default=".")
    parser.add_argument("--policy-config-dir", default=None)
    parser.add_argument("--control-hz", type=float, required=True)
    parser.add_argument("--rate-log-interval-s", type=float, default=None)
    return parser.parse_args()


def _sleep_until_next_tick(tick_start: float, period_s: float) -> None:
    elapsed_s = time.monotonic() - tick_start
    if elapsed_s < period_s:
        time.sleep(period_s - elapsed_s)


def _build_control_command(can_ids: list[int], q_target, *, kp: float, kd: float) -> ControlCommandC:
    if len(can_ids) > MAX_CONTROL_TARGETS:
        raise ValueError(f"too many control targets: {len(can_ids)}/{MAX_CONTROL_TARGETS}")
    command = ControlCommandC()
    command.timestamp_ns = time.time_ns()
    command.num_targets = len(can_ids)
    for index, can_id in enumerate(can_ids):
        target = command.targets[index]
        target.can_id = int(can_id)
        target.q = float(q_target[index])
        target.dq = 0.0
        target.kp = float(kp)
        target.kd = float(kd)
        target.tau = 0.0
    return command


def _parse_can_ids(raw: str) -> list[int]:
    can_ids = [int(part.strip(), 0) for part in raw.split(",") if part.strip()]
    if not can_ids:
        raise ValueError("--can-ids must contain at least one CAN ID")
    if len(set(can_ids)) != len(can_ids):
        raise ValueError("--can-ids must not contain duplicates")
    return can_ids


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    root = project_root(args.project_root)
    runner_raw = load_yaml(args.policy_runner_config)
    section = runner_raw.get("policy_runner")
    if section is None:
        runner_config = runner_raw
    elif isinstance(section, dict):
        runner_config = section
    else:
        raise ValueError("policy_runner config section must be a mapping")

    robot_name = str(args.robot_name)
    policy_config_dir = (
        resolve_policy_config_dir(root, args.policy_config_dir, robot_name)
        if args.policy_config_dir
        else args.policy_list.parent
    )
    policies = load_policies(root, policy_config_dir)
    active_policy_name = runner_config.get("active_policy")
    active_policy = (
        next((policy for policy in policies if policy.name == str(active_policy_name)), None)
        if active_policy_name
        else policies[0]
    )
    if active_policy is None:
        raise ValueError(f"active policy not found: {active_policy_name}")
    pd_config = load_yaml(args.pd_config)
    kp = float(pd_config["kp"])
    kd = float(pd_config["kd"])

    can_ids = _parse_can_ids(args.can_ids)

    control_state_reader = RobotStateShm.open(args.control_state_shm_name)
    aux_reader = AuxCommandShm.open(args.aux_command_shm_name)
    control_command_writer = ControlCommandShm.open(args.mit_command_shm_name)
    print(
        f"[task_controller] control={args.control_state_shm_name} "
        f"aux={args.aux_command_shm_name} control_cmd={args.mit_command_shm_name}",
        flush=True,
    )

    control_hz = float(args.control_hz)
    if control_hz <= 0.0:
        raise ValueError("--control-hz must be > 0")
    rate_log_interval_s = args.rate_log_interval_s
    if rate_log_interval_s is not None and float(rate_log_interval_s) < 0.0:
        raise ValueError("--rate-log-interval-s must be >= 0")
    rate_log_interval_s = None if rate_log_interval_s is None else float(rate_log_interval_s)
    period_s = 1.0 / control_hz
    print(
        f"[task_controller] target_policy_output_hz={control_hz:.3f} "
        f"rate_log_interval_s={rate_log_interval_s}",
        flush=True,
    )
    try:
        print("[task_controller] waiting for control_state", flush=True)
        while RUNNING:
            control_state = control_state_reader.read_relaxed()
            if control_state.is_initialized():
                print("[task_controller] control_state received", flush=True)
                break
            time.sleep(period_s)

        published_count = 0
        last_rate_report_t = time.monotonic()
        last_rate_report_count = 0
        while RUNNING:
            tick_start = time.monotonic()

            control_state = control_state_reader.read_relaxed()
            if not control_state.is_initialized():
                _sleep_until_next_tick(tick_start, period_s)
                continue

            aux_state = aux_reader.read_relaxed()
            lin_vel = [float(value) for value in aux_state.lin_vel_target]
            ang_vel_cmd = [float(value) for value in aux_state.ang_vel_target]
            buttons = mask_to_buttons(int(aux_state.button_mask))

            actuators = {int(item.can_id): item for item in control_state.valid_actuators()}
            dof_pos = np.asarray(
                [float(actuators[can_id].position_rad) for can_id in can_ids],
                dtype=np.float32,
            )
            dof_vel = np.asarray(
                [float(actuators[can_id].velocity_rad_s) for can_id in can_ids],
                dtype=np.float32,
            )
            imu = control_state.imu
            quat = [float(value) for value in imu.quat_wxyz]
            gyro = np.asarray(
                [float(value) for value in imu.angular_velocity_rad_s],
                dtype=np.float32,
            )

            mode = bool(buttons.get("a_button", False))
            active_policy.set_state(dof_pos, dof_vel, quat, gyro)
            active_policy.set_commands(float(lin_vel[0]), float(lin_vel[1]), float(ang_vel_cmd[2]), mode)
            q_target = (active_policy.compute_action()*mode) + action_offset(active_policy, robot_name, dof_pos, mode)

            control_command_writer.write(_build_control_command(can_ids, q_target, kp=kp, kd=kd))
            published_count += 1

            now = time.monotonic()
            if rate_log_interval_s and now - last_rate_report_t >= rate_log_interval_s:
                dt_s = now - last_rate_report_t
                delta_count = published_count - last_rate_report_count
                actual_hz = delta_count / dt_s
                print(
                    f"[task_controller] policy_output_rate_hz={actual_hz:.2f} "
                    f"published={published_count} target_hz={control_hz:.2f}",
                    flush=True,
                )
                last_rate_report_t = now
                last_rate_report_count = published_count

            elapsed_s = now - tick_start
            if elapsed_s >= period_s:
                print(f"[task_controller] loop overrun: {elapsed_s:.6f}s", flush=True)
            _sleep_until_next_tick(tick_start, period_s)
    finally:
        control_state_reader.close()
        aux_reader.close()
        control_command_writer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
