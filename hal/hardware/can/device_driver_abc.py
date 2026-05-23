from abc import ABC, abstractmethod

from hal.can_bus import CANFrame


class CANDeviceDriverBase(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def rx_can_ids(self) -> list[int]:
        raise NotImplementedError

    @abstractmethod
    def on_frame(self, frame: CANFrame) -> None:
        raise NotImplementedError

    def make_periodic_frames(self) -> list[CANFrame]:
        return []

    def update_fault_flags(self) -> None:
        pass