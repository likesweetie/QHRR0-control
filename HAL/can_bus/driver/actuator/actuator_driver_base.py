from abc import ABC, abstractmethod
from dataclasses import dataclass
import time

from can.base import CANFrame


@dataclass
class MotorState:
    position: float = 0.0      # rad
    velocity: float = 0.0      # rad/s
    current: float = 0.0        # A
    temperature: float = 0.0   # degC
    fault_code: int = 0
    timestamp: float = 0.0


class MotorDriverBase(ABC):
    def __init__(self, name: str):
        self.name = name
        self.state = MotorState()

    @abstractmethod
    def feedback_can_ids(self) -> list[int]:
        """
        Dispatcher가 어떤 CAN ID를 이 드라이버에 연결해야 하는지 알려줌.
        """
        raise NotImplementedError

    @abstractmethod
    def on_frame(self, frame: CANFrame) -> None:
        """
        수신 프레임을 해석해서 self.state를 갱신.
        """
        raise NotImplementedError

    @abstractmethod
    def make_enable_frame(self) -> CANFrame:
        raise NotImplementedError

    @abstractmethod
    def make_disable_frame(self) -> CANFrame:
        raise NotImplementedError

    @abstractmethod
    def make_torque_command_frame(self, torque_nm: float) -> CANFrame:
        raise NotImplementedError

    @abstractmethod
    def make_position_command_frame(self, position_rad: float, velocity_rad_s: float = 0.0) -> CANFrame:
        raise NotImplementedError

    @abstractmethod
    def make_clear_fault_frame(self) -> CANFrame:
        raise NotImplementedError

    def get_state(self) -> MotorState:
        return self.state