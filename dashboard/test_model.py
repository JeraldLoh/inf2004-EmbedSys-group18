import unittest
from model import RobotState, validate_command

class StateTests(unittest.TestCase):
    def test_compact_control_and_legacy_vectors(self):
        s = RobotState(broker_connected=True)
        s.receive("heartbeat", bytes.fromhex("b1010501ffffffff78563412"), 4)
        self.assertEqual(s.status(18.9), "online")
        result = bytes.fromhex("b1010614fffffffffeffffff" + "00" * 14)
        s.receive("command_result", result, 5)
        self.assertEqual(s.result["command_id"], 0xffffffff)
        self.assertEqual(s.result["map_revision"], 0xfffffffe)
        broken = bytearray(result)
        broken[12] = 9
        with self.assertRaises(ValueError):
            s.receive("command_result", broken, 6)
        self.assertEqual(s.result["map_revision"], 0xfffffffe)
        s.receive("command_result", b'{"schema":1,"demo":false,"status":"queued"}', 7)
        self.assertEqual(s.result["status"], "queued")
        self.assertEqual(s.status(19), "offline")
    def test_no_heartbeat_and_exact_timeout(self):
        s = RobotState()
        self.assertEqual(s.status(0), "broker disconnected")
        s.broker_connected = True
        self.assertEqual(s.status(0), "waiting for heartbeat")
        s.receive("heartbeat", b'{"schema":1,"demo":true,"seq":1}', 10)
        self.assertEqual(s.status(24.999), "online")
        self.assertEqual(s.status(25), "offline")
    def test_telemetry_does_not_refresh_heartbeat(self):
        s = RobotState(broker_connected=True)
        s.receive("heartbeat", b'{"schema":1,"demo":true,"seq":1}', 0)
        s.receive("telemetry", b'{"schema":1,"demo":true,"speed_percent":20,"distance_cm":80}', 14)
        self.assertEqual(s.status(15), "offline")
    def test_malformed_messages(self):
        s = RobotState()
        for payload in (b'no', b'[]', b'{"schema":1}', b'{"schema":1,"demo":true,"seq":true}'):
            with self.assertRaises(ValueError): s.receive("heartbeat", payload, 0)
        self.assertIsNone(s.last_heartbeat)
    def test_disconnect_and_reconnect_require_new_heartbeat(self):
        s = RobotState(broker_connected=True)
        s.receive("heartbeat", b'{"schema":1,"demo":true,"seq":0}', 0)
        s.disconnect()
        self.assertEqual(s.status(1), "broker disconnected")
        s.broker_connected = True
        self.assertEqual(s.status(1), "waiting for heartbeat")
    def test_commands(self):
        for cmd in ("START", "STOP"):
            self.assertEqual(validate_command(cmd), cmd)
        for cmd in ("", "start", "SET_SPEED 0", "SET_SPEED 100", "SET_SPEED -1", "SET_SPEED 101", "SET_SPEED 2x", "STOP now", "START\x00"):
            with self.assertRaises(ValueError): validate_command(cmd)

if __name__ == "__main__": unittest.main()
