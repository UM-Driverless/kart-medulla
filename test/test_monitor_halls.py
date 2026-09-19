import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location(
    "monitor_halls", Path(__file__).parents[1] / "tools" / "monitor_halls.py")
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def frame(payload, kind=0x0B):
    data = struct.pack(f">{len(payload)}i", *payload)
    body = bytes([len(data), kind]) + data
    return b"\xaa" + body + bytes([monitor.crc8(body)])


class HallMonitorTests(unittest.TestCase):
    def test_crc_reference(self):
        self.assertEqual(monitor.crc8(b"123456789"), 0xF4)

    def test_fragmented_frames_noise_and_unsigned_counts(self):
        payload = [0] * 7 + [0, 5, -1, 2, 3, 400, 1200, 0]
        wire = frame(payload)
        buffer = bytearray(b"boot\n" + wire[:12])
        self.assertEqual(list(monitor.frames(buffer)), [])
        buffer.extend(wire[12:] + frame([1], 0x08))
        self.assertEqual(list(monitor.frames(buffer)), [(0x0B, tuple(payload)), (8, (1,))])
        text = monitor.describe(payload)
        self.assertIn("H1/H2/H3=1/0/1", text)
        self.assertIn("4294967295/2/3", text)

    def test_bad_crc_and_bad_length_recover(self):
        bad = bytearray(frame([0] * 15))
        bad[-1] ^= 1
        buffer = bytearray(b"\xaa\xff\x0b") + bad + frame([0] * 7)
        self.assertEqual(list(monitor.frames(buffer)), [(0x0B, (0,) * 7)])

    def test_unavailable_old_idle_and_invalid(self):
        self.assertIn("no Hall diagnostics", monitor.describe([0] * 7))
        self.assertIn("unavailable", monitor.describe([0] * 7 + [0x106, -1, 0, 0, 0, -1, -1, 0]))
        self.assertIn("no edges yet", monitor.describe([0] * 7 + [0, 0, 0, 0, 0, -1, -1, 0]))
        self.assertIn("Invalid", monitor.describe([0] * 7 + [0, 8, 0, 0, 0, -1, -1, 0]))


if __name__ == "__main__":
    unittest.main()
