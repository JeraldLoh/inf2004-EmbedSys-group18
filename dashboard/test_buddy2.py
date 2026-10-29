import unittest
from pathlib import Path
from binary_codec import decode_telemetry
from model import encode_command, validate_command
class Buddy2Tests(unittest.TestCase):
    def test_c_v3_packet(self):
        data=bytes.fromhex((Path(__file__).resolve().parents[1]/'host_tests/binary-golden-v3.hex').read_text())
        v=decode_telemetry(data)
        self.assertEqual(v['schema'],3)
        self.assertEqual(v['buddies'][2]['left_speed_mm_s'],-10)
        self.assertEqual(v['buddies'][2]['distance_mm'],-500)
        self.assertEqual(v['fault_mask'],0x80000)
    def test_start_stop_only(self):
        self.assertEqual(encode_command('START',0x12345678),bytes([0xb1,2,3,1,0x78,0x56,0x34,0x12]))
        with self.assertRaises(ValueError):validate_command('SET_SPEED 30')
        with self.assertRaises(ValueError):encode_command('START',0)
