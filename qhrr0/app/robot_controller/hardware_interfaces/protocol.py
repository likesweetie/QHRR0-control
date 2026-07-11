from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Protocol, runtime_checkable


# ---------------------------------------------------------------------------
# Controller-facing common state
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Freshness:
    """
    RobotController가 사용하는 데이터 freshness snapshot.

    이 타입은 hardware 구현이나 communication manager에 의존하지 않는다.
    """

    online: bool = False
    stale: bool = True

    last_update_t: float = 0.0
    age_s: float = float("inf")

    rx_count: int = 0
    timeout_count: int = 0
    decode_error_count: int = 0


# ---------------------------------------------------------------------------
# Actuator contract data
# ---------------------------------------------------------------------------


class ActuatorCommandMode(Enum):
    DISABLE = auto()
    ENABLE = auto()


@dataclass(frozen=True, slots=True)
class ActuatorCommand:
    """
    RobotController가 actuator에 전달하는 일반화된 명령.

    장치별 driver는 이 명령을 MIT frame, EtherCAT PDO 또는
    simulation command 등으로 변환한다.
    """

    mode: ActuatorCommandMode

    position_rad: float = 0.0
    velocity_rad_s: float = 0.0

    kp: float = 0.0
    kd: float = 0.0

    torque_nm: float = 0.0


@dataclass(frozen=True, slots=True)
class ActuatorState:
    """
    RobotController가 필요로 하는 actuator 상태 snapshot.
    """

    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0

    enabled: bool = False

    faulted: bool = False
    last_fault_code: int = 0

    freshness: Freshness = field(default_factory=Freshness)


@runtime_checkable
class ControllerActuator(Protocol):
    """
    RobotController가 actuator 객체에 요구하는 최소 계약.

    구현체의 driver, transport, CAN ID, IPC client 구성은
    이 인터페이스에 노출되지 않는다.
    """

    @property
    def name(self) -> str:
        """RobotController 내부에서 사용하는 고유 장치 이름."""
        ...

    def read_state(self, now: float) -> ActuatorState:
        """
        최신 actuator 상태 snapshot을 반환한다.

        계약:
        - blocking 통신을 수행하지 않는다.
        - 내부에 저장된 최신 상태를 반환한다.
        - now를 기준으로 freshness를 반영한다.
        """
        ...

    def write_command(self, command: ActuatorCommand) -> None:
        """
        actuator 명령을 전달한다.

        계약:
        - 장치 응답을 기다리며 block하지 않는다.
        - 실제 장치 상태는 이후 feedback으로 갱신한다.
        """
        ...


# ---------------------------------------------------------------------------
# IMU contract data
# ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ImuState:
    """
    RobotController가 필요로 하는 IMU 상태 snapshot.

    """

    quat_wxyz: tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )

    angular_velocity_rad_s: tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )

    projected_gravity_b: tuple[float, float, float] = (
        0.0,
        0.0,
        -1.0,
    )

    orientation_freshness: Freshness = field(
        default_factory=Freshness,
    )

    gyro_freshness: Freshness = field(
        default_factory=Freshness,
    )


@runtime_checkable
class ControllerImu(Protocol):
    """
    RobotController가 IMU 객체에 요구하는 최소 계약.
    """

    @property
    def name(self) -> str:
        """RobotController 내부에서 사용하는 고유 장치 이름."""
        ...

    def read_state(self, now: float) -> ImuState:
        """
        최신 IMU 상태 snapshot을 반환한다.

        계약:
        - blocking 통신을 수행하지 않는다.
        - 내부에 저장된 최신 상태를 반환한다.
        - orientation과 gyro freshness를 각각 반영한다.
        """
        ...

    def poll(self, now: float) -> None:
        """
        필요한 경우 IMU 데이터 갱신 요청을 보낸다.

        Streaming IMU에서는 no-op이어도 된다.
        Request-response IMU에서는 주기에 맞춰 요청을 전송한다.
        응답을 기다리며 block해서는 안 된다.
        """
        ...


__all__ = [
    "Freshness",
    "ActuatorCommandMode",
    "ActuatorCommand",
    "ActuatorState",
    "ImuState",
    "ControllerActuator",
    "ControllerImu",
]