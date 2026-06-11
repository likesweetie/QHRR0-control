from __future__ import annotations

import struct
import unittest

from qhrr0.app.robot_controller.shm.buffer import DoubleBuffer, PlainBuffer, SeqLockBuffer


class PlainBufferTest(unittest.TestCase):
    def test_round_trip(self) -> None:
        backend = PlainBuffer(payload_size=4)
        buf = bytearray(backend.total_size)

        backend.write(buf, b"ABCD")

        self.assertEqual(backend.read_relaxed(buf), b"ABCD")
        with self.assertRaises(NotImplementedError):
            backend.read_consistent(buf)

    def test_rejects_wrong_payload_size(self) -> None:
        backend = PlainBuffer(payload_size=4)
        with self.assertRaises(ValueError):
            backend.write(bytearray(backend.total_size), b"ABC")


class SeqLockBufferTest(unittest.TestCase):
    def test_consistent_read_round_trip(self) -> None:
        backend = SeqLockBuffer(payload_size=4)
        buf = bytearray(backend.total_size)

        backend.write(buf, b"ABCD")

        self.assertEqual(backend.read_relaxed(buf), b"ABCD")
        self.assertEqual(backend.read_consistent(buf), b"ABCD")
        self.assertEqual(struct.unpack_from("<Q", buf, 0)[0] % 2, 0)

    def test_consistent_read_fails_while_sequence_is_odd(self) -> None:
        backend = SeqLockBuffer(payload_size=4)
        buf = bytearray(backend.total_size)
        struct.pack_into("<Q", buf, 0, 1)

        with self.assertRaises(RuntimeError):
            backend.read_consistent(buf, max_retries=2)


class DoubleBufferTest(unittest.TestCase):
    def test_write_flips_active_slot(self) -> None:
        backend = DoubleBuffer(payload_size=4)
        buf = bytearray(backend.total_size)

        backend.clear(buf)
        self.assertEqual(struct.unpack_from("<I", buf, 0)[0], 0)

        backend.write(buf, b"ABCD")
        self.assertEqual(struct.unpack_from("<I", buf, 0)[0], 1)
        self.assertEqual(backend.read_consistent(buf), b"ABCD")

        backend.write(buf, b"WXYZ")
        self.assertEqual(struct.unpack_from("<I", buf, 0)[0], 0)
        self.assertEqual(backend.read_relaxed(buf), b"WXYZ")

    def test_consistent_read_fails_when_active_slot_is_busy(self) -> None:
        backend = DoubleBuffer(payload_size=4)
        buf = bytearray(backend.total_size)
        backend.clear(buf)
        struct.pack_into("<Q", buf, backend.HEADER_SIZE, 1)

        with self.assertRaises(RuntimeError):
            backend.read_consistent(buf, max_retries=2)


if __name__ == "__main__":
    unittest.main()
