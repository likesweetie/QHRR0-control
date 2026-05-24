#!/usr/bin/env python3

import signal
import threading
import time
from typing import Dict, Optional

from app_config import AppConfig
from imu_protocol import IMUCanClient
from ipc_data import IPCData, create_ipc_data, set_imu_state, set_motor_state
from motor_protocol import *


#82.5 , -96, 20.37 ~ 159.93]
g_run = True

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

CAN_IDS = [0x141, 0x142, 0x143]
MOTOR_OFFSETS = [82.5 * DEG2RAD, -96 * DEG2RAD, 159.93 * DEG2RAD] 


def dps_to_radps(dps: float) -> float:
    return dps * DEG2RAD


def sigint_handler(signum, frame):
    del signum, frame
    global g_run
    g_run = False

def wrap_to_pi(x):
    return (x + math.pi) % (2*math.pi) - math.pi


def run_can_daemon(ipc: Optional[IPCData] = None, app_cfg: Optional[AppConfig] = None) -> IPCData:
    signal.signal(signal.SIGINT, sigint_handler)
    local_ipc = ipc if ipc is not None else create_ipc_data(CAN_IDS)
    cfg_src = app_cfg.daemon if app_cfg is not None else None

    cfg = MITConfig()
    motor_sock = open_can(IFACE)
    imu_client = IMUCanClient(IFACE)

    hz_cmd = 200.0
    dt_cmd = 1.0 / hz_cmd

    target_map = {mid: 0.0 for mid in CAN_IDS}
    v_des_map: Dict[int, float] = {mid: 0.0 for mid in CAN_IDS}
    kp_map: Dict[int, float] = {mid: 0.0 for mid in CAN_IDS}
    kd_map: Dict[int, float] = {mid: 1.0 for mid in CAN_IDS}
    tau_ff_map: Dict[int, float] = {mid: 0.0 for mid in CAN_IDS}
    target_lock = threading.Lock()

    last_loop_start = None
    cycle_count = 0

    try:
        offsets = list(MOTOR_OFFSETS)
        if cfg_src is not None and len(cfg_src.motor_offsets_deg) == len(CAN_IDS):
            offsets = [wrap_to_pi(float(v)) for v in cfg_src.motor_offsets_deg]
        for i, can_id in enumerate(CAN_IDS):
            send_mit_zero(motor_sock, can_id, RAD2DEG * offsets[i])
            time.sleep(0.01)

        for can_id in CAN_IDS:
            send_mit_enter(motor_sock, can_id)
            time.sleep(0.01)

        # # Seed IPC once right after MIT enter so launcher readiness check
        # # is not blocked when 0xC0 status frames are delayed.
        
        with target_lock:
            for can_id in CAN_IDS:
                target_map[can_id] = 0.0
                v_des_map[can_id] = 0.0
                kp_map[can_id] = 0.0
                kd_map[can_id] = 0.0
                tau_ff_map[can_id] = 0.0
                
        send_all_mit(
                sock=motor_sock,
                can_ids=CAN_IDS,
                target_map=target_map,
                v_des_map=v_des_map,
                kp_map=kp_map,
                kd_map=kd_map,
                tau_ff_map=tau_ff_map,
                cfg=cfg
            )

        latest_status, _ = drain_rx_buffer(
            sock=motor_sock,
            can_ids=CAN_IDS,
            expected_cmd=0xC0,
            max_frames=32,
            timeout=1/(hz_cmd)
        )
        
        wall_t = time.time()
        print("init")
        for can_id, st in latest_status.items():
            print("can_id")
            print((math.pi/180)*u16_to_deg(st.enc_u16))
            set_motor_state(
                local_ipc,
                can_id,
                temp_c=st.temp_c,
                iq_a=(st.iq_counts / 2048.0) * 33.0,
                speed_dps=dps_to_radps(float(st.speed_dps)),
                enc_rad=(math.pi/180)*u16_to_deg(st.enc_u16),
                updated_at=wall_t,
            )
            

        next_cmd_t = time.perf_counter()

        while g_run and local_ipc.control.run.value:
            now = time.perf_counter()
            if now < next_cmd_t:
                time.sleep(next_cmd_t - now)

            loop_start = time.perf_counter()
            cycle_count += 1

            if last_loop_start is None:
                actual_period = dt_cmd
            else:
                actual_period = loop_start - last_loop_start
            last_loop_start = loop_start

            local_ipc.control.loop_hz.value = 1.0 / actual_period if actual_period > 0.0 else 0.0
            local_ipc.control.cycle_count.value = cycle_count

            req_zero_seq = int(local_ipc.control.zero_set_request_seq.value)
            if req_zero_seq:
                for i, can_id in enumerate(CAN_IDS):
                    send_mit_zero(motor_sock, can_id, offsets[i])
                    time.sleep(0.01)
                local_ipc.control.zero_set_request_seq.value = 0
                print(f"[daemon] zero set applied (seq={req_zero_seq}).")

            with target_lock:
                if local_ipc.control.damping_enabled.value:
                    for can_id in CAN_IDS:
                        shared_cmd = local_ipc.commands[can_id]
                        target_map[can_id] = float(shared_cmd.pos_rad.value)
                        v_des_map[can_id] = float(shared_cmd.vel_radps.value)
                        kp_map[can_id] = float(shared_cmd.kp.value)
                        kd_map[can_id] = float(shared_cmd.kd.value)
                        tau_ff_map[can_id] = float(shared_cmd.tau_ff.value)
                else:
                    # Keep sending neutral MIT frames so we still receive status feedback.
                    for can_id in CAN_IDS:
                        target_map[can_id] = 0.0
                        v_des_map[can_id] = 0.0
                        kp_map[can_id] = 0.0
                        kd_map[can_id] = 1.0
                        tau_ff_map[can_id] = 0.0


            send_all_mit(
                sock=motor_sock,
                can_ids=CAN_IDS,
                target_map=target_map,
                v_des_map=v_des_map,
                kp_map=kp_map,
                kd_map=kd_map,
                tau_ff_map=tau_ff_map,
                cfg=cfg
            )

            latest_status, _ = drain_rx_buffer(
                sock=motor_sock,
                can_ids=CAN_IDS,
                expected_cmd=0xC0,
                max_frames=32,
                timeout=1/(hz_cmd * 10),
            )

            wall_t = time.time()
            for can_id, st in latest_status.items():
                set_motor_state(
                    local_ipc,
                    can_id,
                    temp_c=st.temp_c,
                    iq_a=(st.iq_counts / 2048.0) * 33.0,
                    speed_dps=dps_to_radps(float(st.speed_dps)),
                    enc_rad=(math.pi/180)*u16_to_deg(st.enc_u16),
                    updated_at=wall_t,
                )
                # print((math.pi/180)*u16_to_deg(st.enc_u16))

            imu_client.request_all()
            latest_imu, imu_recv_count = imu_client.drain_rx_buffer(
                max_frames=32, timeout=1/(hz_cmd * 10))

            if (
                imu_recv_count > 0
                and latest_imu.quat is not None
                and latest_imu.gyro is not None
            ):
                set_imu_state(
                    local_ipc,
                    quat_xyzw=latest_imu.quat.normalized_xyzw(),
                    gyro_dps=(
                        latest_imu.gyro.gx_dps,
                        latest_imu.gyro.gy_dps,
                        latest_imu.gyro.gz_dps,
                    ),
                    gravity_xyz=latest_imu.quat.projected_gravity(),
                    updated_at=wall_t,
                )

            if actual_period > dt_cmd * 1.1:
                print(
                    f"[WARN] period slip: {cycle_count} count, {actual_period * 1000.0:.3f} ms "
                    f"> {dt_cmd * 1000.0:.3f} ms"
                )

            next_cmd_t += dt_cmd

    finally:
        local_ipc.control.run.value = 0
        for can_id in CAN_IDS:
            send_mit_exit(motor_sock, can_id)
            print(f"0x{can_id:X} down.")
            time.sleep(0.005)

        time.sleep(0.02)
        imu_client.close()
        motor_sock.close()
        print("\nMIT mode exited safely.")

    return local_ipc


def main():
    run_can_daemon()


if __name__ == "__main__":
    main()
