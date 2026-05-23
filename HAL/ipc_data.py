#!/usr/bin/env python3

import multiprocessing as mp
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Tuple


@dataclass
class SharedMotorState:
    temp_c: mp.Value
    iq_a: mp.Value
    speed_dps: mp.Value
    enc_rad: mp.Value
    updated_at: mp.Value

    @classmethod
    def create(cls) -> "SharedMotorState":
        return cls(
            temp_c=mp.Value("i", 0),
            iq_a=mp.Value("d", 0.0),
            speed_dps=mp.Value("d", 0.0),
            enc_rad=mp.Value("d", 0.0),
            updated_at=mp.Value("d", 0.0),
        )

    def snapshot(self) -> Dict[str, float]:
        return {
            "temp_c": float(self.temp_c.value),
            "iq_a": float(self.iq_a.value),
            "speed_dps": float(self.speed_dps.value),
            "enc_rad": float(self.enc_rad.value),
            "updated_at": float(self.updated_at.value),
        }


@dataclass
class SharedIMUState:
    lock: Any
    qx: mp.Value
    qy: mp.Value
    qz: mp.Value
    qw: mp.Value
    gx_dps: mp.Value
    gy_dps: mp.Value
    gz_dps: mp.Value
    grav_x: mp.Value
    grav_y: mp.Value
    grav_z: mp.Value
    updated_at: mp.Value

    @classmethod
    def create(cls) -> "SharedIMUState":
        return cls(
            lock=mp.Lock(),
            qx=mp.Value("d", 0.0),
            qy=mp.Value("d", 0.0),
            qz=mp.Value("d", 0.0),
            qw=mp.Value("d", 1.0),
            gx_dps=mp.Value("d", 0.0),
            gy_dps=mp.Value("d", 0.0),
            gz_dps=mp.Value("d", 0.0),
            grav_x=mp.Value("d", 0.0),
            grav_y=mp.Value("d", 0.0),
            grav_z=mp.Value("d", -1.0),
            updated_at=mp.Value("d", 0.0),
        )

    def snapshot(self) -> Dict[str, float]:
        with self.lock:
            return {
                "qx": float(self.qx.value),
                "qy": float(self.qy.value),
                "qz": float(self.qz.value),
                "qw": float(self.qw.value),
                "gx_dps": float(self.gx_dps.value),
                "gy_dps": float(self.gy_dps.value),
                "gz_dps": float(self.gz_dps.value),
                "grav_x": float(self.grav_x.value),
                "grav_y": float(self.grav_y.value),
                "grav_z": float(self.grav_z.value),
                "updated_at": float(self.updated_at.value),
            }


@dataclass
class SharedDaemonControl:
    run: mp.Value
    damping_enabled: mp.Value
    loop_hz: mp.Value
    cycle_count: mp.Value
    zero_set_request_seq: mp.Value
    zero_set_done_seq: mp.Value

    @classmethod
    def create(cls) -> "SharedDaemonControl":
        return cls(
            run=mp.Value("b", 1),
            damping_enabled=mp.Value("b", 0),
            loop_hz=mp.Value("d", 0.0),
            cycle_count=mp.Value("q", 0),
            zero_set_request_seq=mp.Value("q", 0),
            zero_set_done_seq=mp.Value("q", 0),
        )


@dataclass
class SharedMotorCommand:
    pos_rad: mp.Value
    vel_radps: mp.Value
    kp: mp.Value
    kd: mp.Value
    tau_ff: mp.Value

    @classmethod
    def create(
        cls,
        *,
        pos_rad: float = 0.0,
        vel_radps: float = 0.0,
        kp: float = 0.0,
        kd: float = 1.0,
        tau_ff: float = 0.0,
    ) -> "SharedMotorCommand":
        return cls(
            pos_rad=mp.Value("d", float(pos_rad)),
            vel_radps=mp.Value("d", float(vel_radps)),
            kp=mp.Value("d", float(kp)),
            kd=mp.Value("d", float(kd)),
            tau_ff=mp.Value("d", float(tau_ff)),
        )


@dataclass
class IPCData:
    control: SharedDaemonControl
    motors: Dict[int, SharedMotorState]
    imu: SharedIMUState
    commands: Dict[int, SharedMotorCommand]


def create_ipc_data(motor_ids: Iterable[int]) -> IPCData:
    ids = list(motor_ids)
    return IPCData(
        control=SharedDaemonControl.create(),
        motors={motor_id: SharedMotorState.create() for motor_id in ids},
        imu=SharedIMUState.create(),
        commands={motor_id: SharedMotorCommand.create() for motor_id in ids},
    )


def set_motor_state(
    ipc: IPCData,
    motor_id: int,
    *,
    temp_c: float,
    iq_a: float,
    speed_dps: float,
    enc_rad: float,
    updated_at: float,
) -> None:
    shared = ipc.motors.get(motor_id)
    if shared is None:
        return

    shared.temp_c.value = int(temp_c)
    shared.iq_a.value = float(iq_a)
    shared.speed_dps.value = float(speed_dps)
    shared.enc_rad.value = float(enc_rad)
    shared.updated_at.value = float(updated_at)
    

def set_imu_state(
    ipc: IPCData,
    *,
    quat_xyzw: Tuple[float, float, float, float],
    gyro_dps: Tuple[float, float, float],
    gravity_xyz: Tuple[float, float, float],
    updated_at: float,
) -> None:
    qx, qy, qz, qw = quat_xyzw
    gx, gy, gz = gyro_dps
    grav_x, grav_y, grav_z = gravity_xyz

    with ipc.imu.lock:
        ipc.imu.qx.value = float(qx)
        ipc.imu.qy.value = float(qy)
        ipc.imu.qz.value = float(qz)
        ipc.imu.qw.value = float(qw)
        ipc.imu.gx_dps.value = float(gx)
        ipc.imu.gy_dps.value = float(gy)
        ipc.imu.gz_dps.value = float(gz)
        ipc.imu.grav_x.value = float(grav_x)
        ipc.imu.grav_y.value = float(grav_y)
        ipc.imu.grav_z.value = float(grav_z)
        ipc.imu.updated_at.value = float(updated_at)


def set_motor_command(
    ipc: IPCData,
    motor_id: int,
    *,
    pos_rad: float,
    vel_radps: float,
    kp: float,
    kd: float,
    tau_ff: float,
) -> None:
    shared = ipc.commands.get(motor_id)
    if shared is None:
        return

    shared.pos_rad.value = float(pos_rad)
    shared.vel_radps.value = float(vel_radps)
    shared.kp.value = float(kp)
    shared.kd.value = float(kd)
    shared.tau_ff.value = float(tau_ff)
