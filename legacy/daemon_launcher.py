#!/usr/bin/env python3

import multiprocessing as mp
import signal
import time
from typing import Iterable, Optional

from app_config import AppConfig, load_app_config
from can_daemon import CAN_IDS, run_can_daemon
from ipc_data import IPCData, create_ipc_data
from task_controller import run_task_controller


def daemon_process_entry(ipc: IPCData, app_cfg: AppConfig) -> None:
    run_can_daemon(ipc, app_cfg)


def monitor_process_entry(
    ipc: IPCData,
    can_ids: Iterable[int],
    period_s: float = 1.0,
    prompt_done_event: Optional[object] = None,
) -> None:
    if prompt_done_event is not None:
        while ipc.control.run.value and (not prompt_done_event.is_set()):
            time.sleep(0.05)

    while ipc.control.run.value:
        print(f"\n[monitor] loop_hz={ipc.control.loop_hz.value:7.2f}, cycle={ipc.control.cycle_count.value}")

        for can_id in can_ids:
            motor = ipc.motors[can_id]
            motor_s = motor.snapshot()
            print(
                f"  motor=0x{can_id:X} temp={motor_s['temp_c']:3.0f}C "
                f"iq={motor_s['iq_a']:7.3f} speed={motor_s['speed_dps']:7.4f} "
                f"enc={motor_s['enc_rad']:7.4f}"
            )

        imu_s = ipc.imu.snapshot()
        print(
            f"  imu q=({imu_s['qx']:+.4f}, {imu_s['qy']:+.4f}, {imu_s['qz']:+.4f}, {imu_s['qw']:+.4f}) "
            f"proj G=({imu_s['grav_x']:+.4f}, {imu_s['grav_y']:+.4f}, {imu_s['grav_z']:+.4f}) "
            f"gyro=({imu_s['gx_dps']:+.2f}, {imu_s['gy_dps']:+.2f}, {imu_s['gz_dps']:+.2f})"
        )
        time.sleep(period_s)


def wait_for_daemon_ready(ipc: IPCData, can_ids: Iterable[int], timeout_s: float = 5.0) -> bool:
    deadline = time.time() + timeout_s
    # CAN daemon boot / MIT enter sequence 여유 시간
    time.sleep(0.5)

    while time.time() < deadline and ipc.control.run.value:
        if ipc.control.cycle_count.value <= 0:
            time.sleep(0.05)
            continue

        now = time.time()
        all_fresh = True
        for can_id in can_ids:
            updated_at = float(ipc.motors[can_id].updated_at.value)
            print(ipc.motors[can_id].updated_at.value)
            if updated_at <= 0.0 or (now - updated_at) > 1.0:
                all_fresh = False
                break

        if all_fresh:
            return True

        time.sleep(0.05)

    return False


def main() -> None:
    app_cfg = load_app_config()
    ipc = create_ipc_data(CAN_IDS)

    stop_requested = mp.Event()

    def _sigint_handler(signum, frame):
        del signum, frame
        stop_requested.set()
        ipc.control.run.value = 0

    signal.signal(signal.SIGINT, _sigint_handler)

    daemon_proc = mp.Process(
        target=daemon_process_entry,
        args=(ipc, app_cfg),
        name="can_daemon_proc",
    )
    prompt_done_event = mp.Event()

    monitor_proc = None
    if app_cfg.launch.enable_monitor:
        monitor_proc = mp.Process(
            target=monitor_process_entry,
            args=(ipc, CAN_IDS, app_cfg.launch.monitor_period_s, prompt_done_event),
            name="ipc_monitor_proc",
        )

    daemon_proc.start()
    if monitor_proc is not None:
        monitor_proc.start()

    if wait_for_daemon_ready(ipc, CAN_IDS, timeout_s=5.0):
        run_task_controller(
            ipc,
            app_cfg.task_controller,
            CAN_IDS,
            prompt_done_event=prompt_done_event,
            install_signal_handler=False,
        )
    else:
        print("[launcher] daemon ready timeout, starting task_controller anyway.")
        run_task_controller(
            ipc,
            app_cfg.task_controller,
            CAN_IDS,
            prompt_done_event=prompt_done_event,
            install_signal_handler=False,
        )

    try:
        while not stop_requested.is_set():
            if not daemon_proc.is_alive():
                break
            time.sleep(0.1)
    finally:
        ipc.control.run.value = 0

        daemon_proc.join(timeout=3.0)
        if monitor_proc is not None:
            monitor_proc.join(timeout=1.0)

        if daemon_proc.is_alive():
            daemon_proc.terminate()
            daemon_proc.join(timeout=1.0)

        if monitor_proc is not None and monitor_proc.is_alive():
            monitor_proc.terminate()
            monitor_proc.join(timeout=1.0)


if __name__ == "__main__":
    main()
