from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable


if TYPE_CHECKING:
    # 실제 프로젝트의 타입 경로에 맞게 조정하십시오.
    from qhrr0.hardware.actuator.types import (
        ActuatorCommand,
        ActuatorState,
    )
    from qhrr0.hardware.imu.types import ImuState


@runtime_checkable
class ControllerActuator(Protocol):
    """
    RobotController가 actuator에 요구하는 최소 계약.

    구현체는 CAN, EtherCAT, simulation 등 구체적인 transport나
    driver 구현을 외부에 노출하지 않아야 한다.
    """

    @property
    def name(self) -> str:
        """로봇 내부에서 장치를 식별하는 고유 이름."""
        ...

    def read_state(self, now: float) -> ActuatorState:
        """
        최신 actuator 상태 snapshot을 반환한다.

        요구사항:
        - blocking I/O를 수행하지 않는다.
        - 내부에 저장된 최신 상태를 반환한다.
        - now를 기준으로 freshness/stale 상태를 반영한다.
        - 반환되는 상태에는 실제 reported enabled 상태가 포함되어야 한다.
        """
        ...

    def write_command(self, command: ActuatorCommand) -> None:
        """
        actuator 명령을 비동기적으로 전달한다.

        요구사항:
        - 장치 응답을 기다리며 block하지 않는다.
        - command를 driver로 encode한 뒤 통신 채널에 전달한다.
        - 실제 enabled 상태는 명령 전송 여부가 아니라 feedback으로 갱신한다.
        """
        ...


@runtime_checkable
class ControllerImu(Protocol):
    """
    RobotController가 IMU에 요구하는 최소 계약.
    """

    @property
    def name(self) -> str:
        """로봇 내부에서 장치를 식별하는 고유 이름."""
        ...

    def read_state(self, now: float) -> ImuState:
        """
        최신 IMU 상태 snapshot을 반환한다.

        요구사항:
        - blocking I/O를 수행하지 않는다.
        - quaternion, gyro 등 각 데이터 스트림의 freshness를 반영한다.
        - 내부에 저장된 최신 상태를 반환한다.
        """
        ...

    def poll(self, now: float) -> None:
        """
        필요한 경우 IMU 갱신 요청을 비동기적으로 전송한다.

        Streaming IMU는 아무 작업도 하지 않아도 된다.
        Request-response 방식 IMU는 설정된 주기에 맞춰 요청 frame을 보낸다.

        이 메서드는 응답을 기다리며 block해서는 안 된다.
        """
        ...