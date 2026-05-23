from abc import ABC, abstractmethod

from hal.can_bus import CANFrame
from .state import IMUState


class IMUProtocolBase(ABC):
    @abstractmethod
    def rx_can_ids(self) -> list[int]:
        raise NotImplementedError

    @abstractmethod
    def encode_request_quat(self) -> CANFrame:
        raise NotImplementedError

    @abstractmethod
    def encode_request_gyro(self) -> CANFrame:
        raise NotImplementedError

    @abstractmethod
    def encode_request_all(self) -> CANFrame:
        raise NotImplementedError

    @abstractmethod
    def decode_frame(self, frame: CANFrame) -> IMUState | None:
        raise NotImplementedError