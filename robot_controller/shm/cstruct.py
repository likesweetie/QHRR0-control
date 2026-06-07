from __future__ import annotations

import ctypes
from multiprocessing import shared_memory
from typing import ClassVar, Generic, TypeVar


CT = TypeVar("CT", bound=ctypes.Structure)


def struct_size(struct_type: type[ctypes.Structure]) -> int:
    return ctypes.sizeof(struct_type)


def clear_buffer(buf) -> None:
    buf[:] = b"\x00" * len(buf)


class CStructShm(Generic[CT]):
    struct_type: ClassVar[type[CT]]

    def __init__(self, name: str, *, create: bool = False, size: int | None = None) -> None:
        self.name = str(name)
        required_size = self.size_bytes()
        requested_size = required_size if size is None else int(size)
        if requested_size < required_size:
            raise ValueError(
                f"{type(self).__name__} size for {self.name} is too small: "
                f"{requested_size}/{required_size}"
            )
        self.shm = shared_memory.SharedMemory(
            name=self.name,
            create=bool(create),
            size=requested_size if create else 0,
        )
        if len(self.shm.buf) < required_size:
            actual_size = len(self.shm.buf)
            self.close()
            raise RuntimeError(
                f"{type(self).__name__} segment {self.name} is too small: "
                f"{actual_size}/{required_size}"
            )

    @classmethod
    def size_bytes(cls) -> int:
        return ctypes.sizeof(cls.struct_type)

    @classmethod
    def create(cls, name: str, size: int | None = None):
        shm = cls(name, create=True, size=size)
        shm.clear()
        return shm

    @classmethod
    def open_reader(cls, name: str):
        return cls(name, create=False)

    @classmethod
    def open_writer(cls, name: str):
        return cls(name, create=False)

    def read_relaxed(self) -> CT:
        shm = self._require_shm()
        return self.struct_type.from_buffer_copy(shm.buf[: self.size_bytes()])

    def write_raw(self, value: CT) -> None:
        if not isinstance(value, self.struct_type):
            raise TypeError(
                f"{type(self).__name__}.write() expects {self.struct_type.__name__}, "
                f"got {type(value).__name__}"
            )
        data = bytes(value)
        self._require_shm().buf[: len(data)] = data

    def write(self, value: CT) -> None:
        self.write_raw(value)

    def clear(self) -> None:
        clear_buffer(self._require_shm().buf)

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None

    def unlink(self) -> None:
        if self.shm is not None:
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass
            return
        try:
            shm = shared_memory.SharedMemory(name=self.name, create=False)
        except FileNotFoundError:
            return
        try:
            shm.unlink()
        finally:
            shm.close()

    def _require_shm(self) -> shared_memory.SharedMemory:
        if self.shm is None:
            raise RuntimeError(f"{type(self).__name__} segment {self.name} is closed")
        return self.shm
