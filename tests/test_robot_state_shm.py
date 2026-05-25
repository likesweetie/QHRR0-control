from __future__ import annotations

import struct
import unittest
import uuid
from multiprocessing import shared_memory

from robot_controller.core.robot_state_shm import (
    ROBOT_STATE_HEADER_FMT,
    ROBOT_STATE_MAGIC,
    ROBOT_STATE_VERSION,
    RobotStateShmReader,
    RobotStateShmWriter,
)


class RobotStateShmTest(unittest.TestCase):
    def test_empty_initialized_segment_has_no_payload(self) -> None:
        name = f"qhrr_test_state_{uuid.uuid4().hex}"
        segment = shared_memory.SharedMemory(name=name, create=True, size=4096)
        try:
            segment.buf[:] = b"\x00" * len(segment.buf)
            struct.pack_into(
                ROBOT_STATE_HEADER_FMT,
                segment.buf,
                0,
                ROBOT_STATE_MAGIC,
                ROBOT_STATE_VERSION,
                0,
                0,
                0,
            )
            reader = RobotStateShmReader(name)
            try:
                self.assertIsNone(reader.read_latest())
            finally:
                reader.close()
        finally:
            segment.close()
            segment.unlink()

    def test_writer_publishes_readable_payload(self) -> None:
        name = f"qhrr_test_state_{uuid.uuid4().hex}"
        segment = shared_memory.SharedMemory(name=name, create=True, size=4096)
        try:
            segment.buf[:] = b"\x00" * len(segment.buf)
            writer = RobotStateShmWriter(name, 4096)
            reader = RobotStateShmReader(name)
            try:
                writer.publish({"schema": "test", "value": 7})
                self.assertEqual(reader.read_latest()["value"], 7)
            finally:
                writer.close()
                reader.close()
        finally:
            segment.close()
            segment.unlink()


if __name__ == "__main__":
    unittest.main()
