from abc import ABC, abstractmethod

from qhrr0.app.hal.can_bus import CANFrame

class CANDeviceProtocolBase(ABC):
    @abstractmethod
    def rx_can_ids(self) -> list[int]:
        raise NotImplementedError

    @abstractmethod
    def decode_frame(self, frame: CANFrame):
        raise NotImplementedError