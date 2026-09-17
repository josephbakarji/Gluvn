import queue
import struct
import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from digital_twin import rotation_from_quat
from core.port_read import ParseSerial


class TestNavQuatPipeline(unittest.TestCase):
    def test_rotation_from_quat_reorders_firmware_components(self):
        q = (1.0, 0.0, 0.0, 0.0)
        R = rotation_from_quat(q)
        np = __import__('numpy')
        np.testing.assert_allclose(R, np.eye(3), atol=1e-8)

    def test_parse_serial_decodes_nav_quat_when_enabled(self):
        parser = ParseSerial(
            sensq=queue.Queue(),
            time0=0.0,
            flex=True,
            press=True,
            imu=True,
            nav_quat_enabled=True,
        )

        base = struct.pack(
            ParseSerial._BASE_SAMPLE,
            ord('r'), 1, 1234,
            *([0] * 16),
        )
        nav_q = (16384, -8192, 8192, 0)
        payload = base + struct.pack('>hhhh', *nav_q)
        crc = parser._crc16_ccitt(payload)
        frame = b'\xA5\x5A' + bytes([len(payload)]) + payload + struct.pack('>H', crc)
        parser.sensq.put(frame)
        parser.parse()

        sample = parser.getQ().get(timeout=1.0)
        self.assertIn('nav_quat', sample)
        self.assertEqual(sample['nav_quat'], tuple(v / 32767.0 for v in nav_q))


if __name__ == '__main__':
    unittest.main()
