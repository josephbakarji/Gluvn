import queue
import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.pose_provider import PoseProvider


class _Parser:
    def __init__(self):
        self.queue = queue.Queue()

    def getQ(self):
        return self.queue


class _Reader:
    def __init__(self):
        self.hands = ['r']
        self.commands = []
        self.threads = {'r': {'parser': _Parser()}}

    def start_readers(self):
        pass

    def stop_readers(self):
        pass

    def send_command(self, hand, command):
        self.commands.append((hand, command))


class TestPoseProvider(unittest.TestCase):
    def test_reference_counts_and_independent_streams(self):
        reader = _Reader()
        provider = PoseProvider(reader=reader)
        with provider.request('r', motion=True):
            with provider.request('r', motion=True, orientation=True):
                self.assertEqual(reader.commands, [
                    ('r', 'MOTION_STREAM_ON'),
                    ('r', 'NAV_QUAT_STREAM_ON'),
                ])
            self.assertEqual(reader.commands[-1], ('r', 'NAV_QUAT_STREAM_OFF'))
        self.assertEqual(reader.commands[-1], ('r', 'MOTION_STREAM_OFF'))

    def test_dispatch_returns_decoded_pose_structure(self):
        reader = _Reader()
        provider = PoseProvider(reader=reader)
        provider.start()
        reader.threads['r']['parser'].queue.put({
            'seq': 7,
            'device_us': 1234,
            'position': (1.0, 2.0, 3.0),
            'velocity': (0.1, 0.2, 0.3),
            'nav_quat': (1.0, 0.0, 0.0, 0.0),
        })
        sample = provider.wait_for_sample('r', timeout=1.0)
        provider.stop()
        self.assertEqual(sample.position, (1.0, 2.0, 3.0))
        self.assertEqual(sample.velocity, (0.1, 0.2, 0.3))
        self.assertEqual(sample.orientation, (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(sample.seq, 7)


if __name__ == '__main__':
    unittest.main()
