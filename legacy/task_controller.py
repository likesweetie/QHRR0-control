#!/usr/bin/env python3

import math
import signal
import time
from typing import Dict, Iterable, Optional

from app_config import TaskControllerConfig
from ipc_data import IPCData, set_motor_command

RUN = True


def _sigint_handler(signum, frame):
    del signum, frame
    global RUN
    RUN = False


def _read_motor_angles(ipc: IPCData, can_ids: Iterable[int], timeout_s: float = 5.0) -> Dict[int, float]:
    deadline = time.time() + timeout_s
    mids = list(can_ids)

    while time.time() < deadline and RUN:
        result: Dict[int, float] = {}
        now = time.time()

        for can_id in mids:
            m = ipc.motors[can_id]
            updated = float(m.updated_at.value)
            if updated <= 0.0 or (now - updated) > 1.0:
                result.clear()
                break
            result[can_id] = float(m.enc_rad.value)

        if len(result) == len(mids):
            return result

        time.sleep(0.05)

    raise TimeoutError("IPC CAN state is not ready")


def _write_command(
    ipc: IPCData,
    targets: Dict[int, float],
    cfg: TaskControllerConfig,
    damping_enabled: bool,
) -> None:
    if not damping_enabled:
        ipc.control.damping_enabled.value = 0

    for can_id, pos in targets.items():
        set_motor_command(
            ipc,
            can_id,
            pos_rad=pos,
            vel_radps=cfg.command_v_des,
            kp=cfg.command_kp,
            kd=cfg.command_kd,
            tau_ff=cfg.command_tau_ff,
        )

    if damping_enabled:
        ipc.control.damping_enabled.value = 1


def _request_zero_set(ipc: IPCData, timeout_s: float = 2.0) -> bool:
    req_seq = int(ipc.control.zero_set_request_seq.value) + 1
    ipc.control.zero_set_request_seq.value = 1

    deadline = time.time() + timeout_s
    while time.time() < deadline and RUN and ipc.control.run.value:
        if int(ipc.control.zero_set_done_seq.value) >= req_seq:
            return True
        time.sleep(0.01)
    return False

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


def dps_to_radps(dps: float) -> float:
    return dps * DEG2RAD


def sigint_handler(signum, frame):
    del signum, frame
    global g_run
    g_run = False

def wrap_to_pi(x):
    return (x + math.pi) % (2*math.pi) - math.pi


def run_task_controller(
    ipc: IPCData,
    cfg: TaskControllerConfig,
    can_ids: Iterable[int],
    prompt_done_event: Optional[object] = None,
    install_signal_handler: bool = True,
) -> None:
    global RUN
    RUN = True

    if install_signal_handler:
        signal.signal(signal.SIGINT, _sigint_handler)
    mids = list(can_ids)

    print("[task_controller] waiting for daemon state...")
    try:
        q_preview = _read_motor_angles(ipc, mids, timeout_s=8.0)
    except TimeoutError as exc:
        print(f"[task_controller] {exc}")
        if prompt_done_event is not None:
            prompt_done_event.set()
        return

    print("[task_controller] current hardware angle (rad):")
    for can_id in mids:
        print(f"  motor=0x{can_id:X} q={q_preview[can_id]:+.4f}")

    try:
        answer = input("\nSet current pose as zero reference and move to HOME_POS with 1-cos trajectory? [y/N]: ").strip().lower()
    except EOFError:
        print("[task_controller] stdin is not interactive (EOF).")
        if prompt_done_event is not None:
            prompt_done_event.set()
        return

    if answer not in {"y", "yes"}:
        print("[task_controller] cancelled.")
        if prompt_done_event is not None:
            prompt_done_event.set()
        return

    try:
        q_confirm = _read_motor_angles(ipc, mids, timeout_s=1.0)
    except TimeoutError:
        q_confirm = {mid: float(ipc.motors[mid].enc_rad.value) for mid in mids}
    # _write_command(ipc, q_confirm, cfg, damping_enabled=False)
    print("[task_controller] snapshot written at confirm time.")

    q_home = {mid: float(cfg.home_pos_rad[i]) for i, mid in enumerate(mids)}
    if prompt_done_event is not None:
        prompt_done_event.set()

    ipc.control.damping_enabled.value = 0
    print("[task_controller] requesting zero set...")
    _ = _request_zero_set(ipc, timeout_s=2.0)
    print("[task_controller] zero set sent.")

    settle_s = max(0.0, float(cfg.zero_set_settle_s))
    if settle_s > 0.0:
        print(f"[task_controller] waiting {settle_s:.1f}s before position control...")
        time.sleep(settle_s)

    q_start = _read_motor_angles(ipc, mids, timeout_s=1.0)
    # _write_command(ipc, q_start, cfg, damping_enabled=False)
    print("[task_controller] trajectory start angle updated from current state.")

    hz = max(1.0, float(cfg.trajectory_hz))
    dt = 1.0 / hz
    duration = max(0.1, float(cfg.trajectory_duration_s))

    t0 = time.perf_counter()
    next_t = t0

    while RUN and ipc.control.run.value:
        now = time.perf_counter()
        if now < next_t:
            time.sleep(next_t - now)

        t = time.perf_counter() - t0
        if t >= duration:
            _write_command(ipc, q_home, cfg, damping_enabled=True)
            break

        s = 0.5 * (1.0 - math.cos(math.pi * t / duration))
        q_cmd = {mid: q_start[mid] + wrap_to_pi(q_home[mid] - q_start[mid]) * s for mid in mids}
        print(q_cmd)
        _write_command(ipc, q_cmd, cfg, damping_enabled=True)

        next_t += dt

    print("[task_controller] arrived at HOME_POS and holding.")
    hold_dt = 1.0 / max(1.0, float(cfg.hold_hz))

    while RUN and ipc.control.run.value:
        _write_command(ipc, q_home, cfg, damping_enabled=True)
        time.sleep(hold_dt)


def main() -> None:
    raise RuntimeError("task_controller.py should be started by daemon_launcher.py (shared IPC mode)")


if __name__ == "__main__":
    main()
