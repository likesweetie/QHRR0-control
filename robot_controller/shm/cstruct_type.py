from __future__ import annotations

import ctypes
from multiprocessing import shared_memory
from typing import Any, ClassVar, Generic, Mapping, TypeVar

from robot_controller.shm.buffer import BufferBackend, PlainBuffer


CT = TypeVar("CT", bound=ctypes.Structure)


def struct_size(struct_type: type[ctypes.Structure]) -> int:
    return ctypes.sizeof(struct_type)


def clear_buffer(buf) -> None:
    buf[:] = b"\x00" * len(buf)


class CStructShm(Generic[CT]):
    """Shared-memory channel for a fixed-size ctypes.Structure.

    This class owns the shared-memory resource and delegates the actual byte
    storage policy to a buffer backend.

    Responsibilities:
    - create/open/close/unlink shared memory
    - validate segment size
    - convert ctypes.Structure <-> bytes
    - delegate byte-level read/write/clear to the selected backend

    Non-responsibilities:
    - command construction
    - timestamp generation
    - Data validation
    """

    struct_type: ClassVar[type[CT]]
    default_buffer_backend: ClassVar[type[BufferBackend]] = PlainBuffer

    def __init__(
        self,
        name: str,
        *,
        create: bool = False,
        size: int | None = None,
        buffer_backend: type[BufferBackend] | None = None,
        buffer_options: Mapping[str, Any] | None = None,
    ) -> None:
        self.name = str(name)
        self.buffer_backend = self._make_buffer_backend(
            buffer_backend=buffer_backend,
            buffer_options=buffer_options,
        )

        required_size = self.segment_size_bytes()
        requested_size = required_size if size is None else int(size)

        if requested_size < required_size:
            raise ValueError(
                f"{type(self).__name__} segment size for {self.name} is too small: "
                f"{requested_size}/{required_size}"
            )

        self.shm: shared_memory.SharedMemory | None = shared_memory.SharedMemory(
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
        """Return ctypes payload size in bytes.

        This preserves the old meaning of size_bytes():
        it is the size of the stored ctypes.Structure, not necessarily
        the full shared-memory segment size.
        """
        return ctypes.sizeof(cls.struct_type)

    @classmethod
    def create(
        cls,
        name: str,
        size: int | None = None,
        *,
        buffer_backend: type[BufferBackend] | None = None,
        buffer_options: Mapping[str, Any] | None = None,
    ):
        shm = cls(
            name,
            create=True,
            size=size,
            buffer_backend=buffer_backend,
            buffer_options=buffer_options,
        )
        shm.clear()
        return shm

    @classmethod
    def open_reader(
        cls,
        name: str,
        *,
        buffer_backend: type[BufferBackend] | None = None,
        buffer_options: Mapping[str, Any] | None = None,
    ):
        return cls(
            name,
            create=False,
            buffer_backend=buffer_backend,
            buffer_options=buffer_options,
        )

    @classmethod
    def open_writer(
        cls,
        name: str,
        *,
        buffer_backend: type[BufferBackend] | None = None,
        buffer_options: Mapping[str, Any] | None = None,
    ):
        return cls(
            name,
            create=False,
            buffer_backend=buffer_backend,
            buffer_options=buffer_options,
        )

    def segment_size_bytes(self) -> int:
        """Return actual required shared-memory segment size.

        For PlainBuffer:
            segment_size == ctypes payload size

        For SeqLockBuffer / DoubleBuffer:
            segment_size > ctypes payload size
        """
        return int(self.backend.total_size)

    def read(self, *, consistent: bool = False, max_retries: int = 10) -> CT:
        """Read one structure from shared memory.

        Default behavior is relaxed read.

        Use:
            shm.read()

        for the existing relaxed behavior.

        Use:
            shm.read(consistent=True)

        when the selected backend supports a consistency protocol.
        """
        if consistent:
            return self.read_consistent(max_retries=max_retries)
        return self.read_relaxed()

    def read_relaxed(self) -> CT:
        payload = self.backend.read_relaxed(self._require_shm().buf)
        return self.struct_type.from_buffer_copy(payload)

    def read_consistent(self, *, max_retries: int = 10) -> CT:
        payload = self.backend.read_consistent(
            self._require_shm().buf,
            max_retries=max_retries,
        )
        return self.struct_type.from_buffer_copy(payload)

    def write_raw(self, value: CT) -> None:
        """Write a ctypes.Structure to shared memory.

        Kept for backward compatibility with the previous API.
        """
        self.write(value)

    def write(self, value: CT) -> None:
        if not isinstance(value, self.struct_type):
            raise TypeError(
                f"{type(self).__name__}.write() expects "
                f"{self.struct_type.__name__}, got {type(value).__name__}"
            )

        self.backend.write(self._require_shm().buf, bytes(value))

    def clear(self) -> None:
        self.backend.clear(self._require_shm().buf)

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

    def _make_buffer_backend(
        self,
        *,
        buffer_backend: type[BufferBackend] | None,
        buffer_options: Mapping[str, Any] | None,
    ) -> BufferBackend:
        backend_type = buffer_backend or self.default_buffer_backend
        options = dict(buffer_options or {})
        return backend_type(payload_size=self.size_bytes(), **options)

    def _require_shm(self) -> shared_memory.SharedMemory:
        if self.shm is None:
            raise RuntimeError(f"{type(self).__name__} segment {self.name} is closed")
        return self.shm

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.close()