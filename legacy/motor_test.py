#!/usr/bin/env python3

import math
import signal
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

IFACE = "can0"
FRAME_FMT = "=IB3x8s"

g_run = True


def sigint_handler(signum, frame):
    global g_run
    g_run = False


def open_can(iface: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((iface,))
    return sock


def send_frame(sock: socket.socket, can_id: int, data8: bytes) -> None:
    if len(data8) != 8:
        raise ValueError("CAN payload must be exactly 8 bytes")
    frame = struct.pack(FRAME_FMT, can_id, 8, data8)
    sock.send(frame)


def recv_frame(
    sock: socket.socket, timeout: float = 0.0
) -> Optional[Tuple[int, int, bytes]]:
    sock.settimeout(timeout)
    try:
        frame = sock.recv(16)
    except (TimeoutError, OSError):
        return None

    can_id, dlc, data = struct.unpack(FRAME_FMT, frame)
    return can_id, dlc, data[:dlc]


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = clamp(x, x_min, x_max)
    span = x_max - x_min
    max_int = (1 << bits) - 1
    return int(round((x - x_min) * max_int / span))


@dataclass
class MITConfig:
    p_max: float = 12.5
    v_max: float = 45.0
    kp_max: float = 500.0
    kd_max: float = 5.0
    t_max: float = 33.0


@dataclass
class MotorStatus:
    temp_c: int
    iq_counts: int
    speed_dps: int
    enc_u16: int


def parse_status_common(payload8: bytes, expected_cmd: int) -> MotorStatus:
    if len(payload8) != 8 or payload8[0] != expected_cmd:
        raise ValueError("Invalid response")

    temp_c = struct.unpack("b", payload8[1:2])[0]
    iq_counts = struct.unpack("<h", payload8[2:4])[0]
    speed_dps = struct.unpack("<h", payload8[4:6])[0]
    enc_u16 = struct.unpack("<H", payload8[6:8])[0]

    return MotorStatus(
        temp_c=temp_c,
        iq_counts=iq_counts,
        speed_dps=speed_dps,
        enc_u16=enc_u16,
    )


def send_mit_enter(sock: socket.socket, can_id: int) -> None:
    send_frame(sock, can_id, bytes([0xC1, 0, 0, 0, 0, 0, 0, 0]))


def send_mit_exit(sock: socket.socket, can_id: int) -> None:
    send_frame(sock, can_id, bytes([0xC2, 0, 0, 0, 0, 0, 0, 0]))


def send_mit_zero(sock: socket.socket, can_id: int) -> None:
    send_frame(sock, can_id, bytes([0xC3, 0, 0, 0, 0, 0, 0, 0]))


def pack_mit_payload(
    p_des: float,
    v_des: float,
    kp: float,
    kd: float,
    tau_ff: float,
    cfg: MITConfig,
) -> bytes:
    p_u = float_to_uint(p_des, -cfg.p_max, cfg.p_max, 16)
    v_u = float_to_uint(v_des, -cfg.v_max, cfg.v_max, 12)
    kp_u = float_to_uint(kp, 0.0, cfg.kp_max, 12)
    kd_u = float_to_uint(kd, 0.0, cfg.kd_max, 8)
    t_u = float_to_uint(tau_ff, -cfg.t_max, cfg.t_max, 8)

    data = bytearray(8)
    data[0] = 0xC0
    data[1] = (p_u >> 8) & 0xFF
    data[2] = p_u & 0xFF
    data[3] = (v_u >> 4) & 0xFF
    data[4] = ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)
    data[5] = kp_u & 0xFF
    data[6] = kd_u & 0xFF
    data[7] = t_u & 0xFF
    return bytes(data)


def send_mit_control(
    sock: socket.socket,
    can_id: int,
    p_des: float,
    v_des: float,
    kp: float,
    kd: float,
    tau_ff: float,
    cfg: MITConfig,
) -> None:
    payload = pack_mit_payload(p_des, v_des, kp, kd, tau_ff, cfg)
    send_frame(sock, can_id, payload)


def send_all_mit(
    sock: socket.socket,
    can_ids: List[int],
    target_map: Dict[int, float],
    v_des_map: Dict[int, float],
    kp_map: Dict[int, float],
    kd_map: Dict[int, float],
    tau_ff_map: Dict[int, float],
    cfg: MITConfig,
    target_lock: threading.Lock,
) -> None:
    with target_lock:
        for can_id in can_ids:
            p_des = target_map[can_id]
            v_des = v_des_map[can_id]
            kp = kp_map[can_id]
            kd = kd_map[can_id]
            tau_ff = tau_ff_map[can_id]
            send_mit_control(sock, can_id, p_des, v_des, kp, kd, tau_ff, cfg)


def drain_rx_buffer(
    sock: socket.socket,
    can_ids: List[int],
    expected_cmd: int = 0xC0,
    max_frames: int = 4096,
) -> Tuple[Dict[int, MotorStatus], int]:
    latest_status: Dict[int, MotorStatus] = {}
    recv_count = 0
    can_id_set = set(can_ids)

    for _ in range(max_frames):
        rx = recv_frame(sock, timeout=0.0)
        if rx is None:
            break

        rid, dlc, data = rx

        if rid not in can_id_set:
            continue
        if dlc != 8:
            continue
        if len(data) != 8:
            continue
        if data[0] != expected_cmd:
            continue

        try:
            st = parse_status_common(data, expected_cmd=expected_cmd)
            latest_status[rid] = st
            recv_count += 1
        except ValueError:
            continue

    return latest_status, recv_count


def trajectory_thread_fn(
    can_id_traj: int,
    target_map: Dict[int, float],
    v_des_map: Dict[int, float],
    target_lock: threading.Lock,
    hz_traj: float = 50.0,
    traj_duration: float = 2.0,
    q0: float = 0.0,
    qf: float = -0.1,
):
    """
    50 Hz trajectory thread
    can_id_traj만 zero 기준으로 0 -> -0.1 rad 코사인 보간
    """
    dt = 1.0 / hz_traj
    next_t = time.perf_counter()
    t0 = next_t

    while g_run:
        now = time.perf_counter()
        if now < next_t:
            time.sleep(next_t - now)

        t = time.perf_counter() - t0

        if t < traj_duration:
            s = 0.5 * (1.0 - math.cos(math.pi * t / traj_duration))
            q = q0 + (qf - q0) * s
            dq = (
                (qf - q0)
                * 0.5
                * math.pi
                / traj_duration
                * math.sin(math.pi * t / traj_duration)
            )
        else:
            q = qf
            dq = 0.0

        with target_lock:
            target_map[can_id_traj] = -0.0
            v_des_map[can_id_traj] = 0
            target_map[0x142] = -0.0
            v_des_map[0x142] = 0

        next_t += dt


def main():
    signal.signal(signal.SIGINT, sigint_handler)

    can_ids = [0x141, 0x142, 0x143]
    traj_can_id = 0x143

    cfg = MITConfig()
    sock = open_can(IFACE)

    # -------------------------
    # command loop: 50 Hz
    # receive loop: 1 Hz
    # trajectory loop: 50 Hz
    # -------------------------
    hz_cmd = 50.0
    dt_cmd = 1.0 / hz_cmd

    hz_recv = 1.0
    dt_recv = 1.0 / hz_recv

    # per-actuator targets / gains
    target_map: Dict[int, float] = {mid: 0.0 for mid in can_ids}
    v_des_map: Dict[int, float] = {mid: 0.0 for mid in can_ids}
    kp_map: Dict[int, float] = {mid: 0.0 for mid in can_ids}
    kd_map: Dict[int, float] = {mid: 0.0 for mid in can_ids}
    tau_ff_map: Dict[int, float] = {mid: 0.0 for mid in can_ids}

    target_lock = threading.Lock()

    last_loop_start = None
    cycle_count = 0
    last_recv_t = None

    status_map: Dict[int, MotorStatus] = {}

    traj_thread = None

    try:
        # zero
        for can_id in can_ids:
            send_mit_zero(sock, can_id)
            time.sleep(0.01)

        # enter MIT
        for can_id in can_ids:
            send_mit_enter(sock, can_id)
            time.sleep(0.01)

        # trajectory thread start
        # traj_thread = threading.Thread(
        #     target=trajectory_thread_fn,
        #     args=(
        #         traj_can_id,
        #         target_map,
        #         v_des_map,
        #         target_lock,
        #     ),
        #     kwargs={
        #         "hz_traj": 50.0,
        #         "traj_duration": 0.1,
        #         "q0": 0.0,
        #         "qf": -0.7,
        #     },
        #     daemon=True,
        # )
        # traj_thread.start()

        next_cmd_t = time.perf_counter()
        last_recv_t = next_cmd_t

        while g_run:
            # -------------------------
            # 1) wait until next 50 Hz command slot
            # -------------------------
            now = time.perf_counter()
            if now < next_cmd_t:
                time.sleep(next_cmd_t - now)

            loop_start = time.perf_counter()
            cycle_count += 1

            # -------------------------
            # 2) monitor loop timing
            # -------------------------
            if last_loop_start is None:
                actual_period = dt_cmd
            else:
                actual_period = loop_start - last_loop_start

            lateness = loop_start - next_cmd_t
            last_loop_start = loop_start

            # -------------------------
            # 3) send MIT command to all IDs @ 50 Hz
            # -------------------------
            send_all_mit(
                sock=sock,
                can_ids=can_ids,
                target_map=target_map,
                v_des_map=v_des_map,
                kp_map=kp_map,
                kd_map=kd_map,
                tau_ff_map=tau_ff_map,
                cfg=cfg,
                target_lock=target_lock,
            )

            # -------------------------
            # 4) drain RX buffer @ 1 Hz
            # -------------------------
            did_recv = False
            recv_count = 0
            recv_elapsed_ms = 0.0

            if (loop_start - last_recv_t) >= dt_recv:
                did_recv = True
                rx_start = time.perf_counter()

                latest_status, recv_count = drain_rx_buffer(
                    sock=sock,
                    can_ids=can_ids,
                    expected_cmd=0xC0,
                    max_frames=4096,
                )

                rx_end = time.perf_counter()
                recv_elapsed_ms = (rx_end - rx_start) * 1000.0
                last_recv_t = loop_start

                status_map.update(latest_status)

            # -------------------------
            # 5) print timing + target info
            # -------------------------
            with target_lock:
                q143 = target_map[traj_can_id]
                dq143 = v_des_map[traj_can_id]

            # print(
            #     f"\n[cycle {cycle_count}] "
            #     f"actual_period={actual_period*1000.0:.3f} ms  "
            #     f"target={dt_cmd*1000.0:.3f} ms  "
            #     f"lateness={lateness*1000.0:.3f} ms  "
            #     f"q_des(0x{traj_can_id:X})={q143:+.4f} rad  "
            #     f"dq_des={dq143:+.4f} rad/s"
            # )

            if did_recv:
                print(f"[RX] drained {recv_count} frame(s) in {recv_elapsed_ms:.3f} ms")

                for can_id in can_ids:
                    st = status_map.get(can_id, None)
                    if st is not None:
                        print(
                            f"  can_id=0x{can_id:X} "
                            f"temp={st.temp_c:3d}C  "
                            f"iq={(st.iq_counts / 2048 * 33):5f}  "
                            f"spd_dps={st.speed_dps:5d}  "
                            f"enc={((st.enc_u16) / 16384 * 6.28):3f}"
                        )
                    else:
                        print(f"  can_id=0x{can_id:X}  no cached response")

            # -------------------------
            # 6) period slip warning
            # -------------------------
            if actual_period > dt_cmd * 1.2:
                print(
                    f"[WARN] control period slip detected: "
                    f"{actual_period*1000.0:.3f} ms > {dt_cmd*1000.0:.3f} ms"
                )

            # -------------------------
            # 7) schedule 3next command tick
            # -------------------------
            next_cmd_t += dt_cmd

    finally:
        for can_id in can_ids:
            send_mit_exit(sock, can_id)
            print(f"0x{can_id:X} down.")
            time.sleep(0.005)

        if traj_thread is not None:
            traj_thread.join(timeout=0.2)

        time.sleep(0.02)
        sock.close()
        print("\nMIT mode exited safely.")


if __name__ == "__main__":
    main()