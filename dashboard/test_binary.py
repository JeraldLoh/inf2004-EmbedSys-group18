import unittest
from pathlib import Path
from binary_codec import decode_telemetry
from model import RobotState

class BinaryTests(unittest.TestCase):
    def fixture(self):
        return bytes.fromhex((Path(__file__).resolve().parents[1]/'host_tests/binary-golden.hex').read_text())

    def test_c_fixture(self):
        value = decode_telemetry(self.fixture())
        self.assertEqual(value['buddies'][3]['steering'], -100)
        self.assertEqual(value['buddies'][5]['event_seq'], 0xffffffff)
        self.assertEqual(value['speed_percent'], 42)
        state = RobotState()
        state.receive('telemetry', self.fixture(), 0)
        self.assertEqual(state.telemetry['distance_cm'], 93)

    def test_every_truncation_rejected(self):
        data = self.fixture()
        for n in range(len(data)):
            with self.assertRaises(ValueError):
                decode_telemetry(data[:n])
