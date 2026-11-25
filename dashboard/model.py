"""Heartbeat and message validation independent of MQTT and tkinter."""
from dataclasses import dataclass
import json
from binary_codec import decode_telemetry

def decode_control_wire(topic, payload):
    heartbeat = topic == "heartbeat"
    if not heartbeat and topic != "command_result":
        raise ValueError("Unexpected topic")
    if len(payload) < 4 or payload[:3] != bytes((0xb1, 1, 5 if heartbeat else 6)):
        raise ValueError("Invalid binary control header")
    flags = payload[3]
    if heartbeat:
        if len(payload) != 12 or flags > 1:
            raise ValueError("Invalid binary heartbeat")
        return dict(schema=1, demo=bool(flags), seq=int.from_bytes(payload[4:8], "little"),
                    uptime_ms=int.from_bytes(payload[8:12], "little"))
    statuses = ("simulated", "queued", "rejected", "queue_full", "applied")
    code, has_map = flags & 7, bool(flags & 16)
    if flags & 0xe0 or code >= len(statuses) or has_map != (code == 4) or len(payload) != (26 if has_map else 8):
        raise ValueError("Invalid binary command result")
    value = dict(schema=1, demo=bool(flags & 8), status=statuses[code],
                 command_id=int.from_bytes(payload[4:8], "little"))
    if has_map:
        revision = int.from_bytes(payload[8:12], "little")
        raw_map = int.from_bytes(payload[12:], "little")
        if not value["command_id"] or not revision or revision & 1 or raw_map >> 108:
            raise ValueError("Invalid binary map result")
        entries = [(raw_map >> (index * 3)) & 7 for index in range(36)]
        assigned = [entry for entry in entries if entry]
        if any(entry > 4 for entry in assigned) or len(assigned) != len(set(assigned)):
            raise ValueError("Invalid binary map entries")
        value.update(map=payload[12:].hex(), map_revision=revision)
    return value

def validate_command(command):
    if command in ("START", "STOP"):
        return command
    raise ValueError("Only START and STOP are supported")

def encode_command(command, command_id):
    validate_command(command)
    if type(command_id) is not int or not 1 <= command_id <= 0xffffffff:
        raise ValueError('Invalid command ID')
    return bytes([0xb1,2,3,1 if command=='START' else 2])+command_id.to_bytes(4,'little')

@dataclass
class RobotState:
    broker_connected: bool = False
    last_heartbeat: float | None = None
    telemetry: dict | None = None
    telemetry_at: float | None = None
    result: dict | None = None
    demo: bool | None = None

    def disconnect(self):
        self.broker_connected = False
        self.last_heartbeat = None
        self.telemetry = None
        self.telemetry_at = None
        self.result = None
        self.demo = None

    def status(self, now):
        if not self.broker_connected:
            return "broker disconnected"
        if self.last_heartbeat is None:
            return "waiting for heartbeat"
        return "online" if now - self.last_heartbeat < 15 else "offline"

    def receive(self, topic, payload, now):
        if len(payload) > 1024:
            raise ValueError("Message too large")
        try:
            wire = isinstance(payload, (bytes, bytearray)) and payload[:1] == b'\xb1'
            binary = topic == 'telemetry' and wire
            value = decode_telemetry(payload) if binary else decode_control_wire(topic, payload) if wire else json.loads(payload)
        except (ValueError, UnicodeError) as error:
            raise ValueError("Invalid MQTT payload") from error
        if not isinstance(value, dict) or type(value.get("schema")) is not int or value["schema"] not in ((2,3) if binary else (1,)) or type(value.get("demo")) is not bool:
            raise ValueError("Invalid schema/demo field")
        if topic == "heartbeat":
            if type(value.get("seq")) is not int or value["seq"] < 0:
                raise ValueError("Invalid sequence")
            self.last_heartbeat = now
        elif topic == "telemetry":
            for key, maximum in (("speed_percent", 100), ("distance_cm", 100000)):
                if type(value.get(key)) is not int or not 0 <= value[key] <= maximum:
                    raise ValueError("Invalid measurement")
            self.telemetry = value
            self.telemetry_at = now
        elif topic == "command_result":
            if value.get("status") not in ("simulated", "queued", "rejected", "queue_full", "applied"):
                raise ValueError("Invalid command result")
            self.result = value
        else:
            raise ValueError("Unknown topic")
        self.demo = value["demo"]
