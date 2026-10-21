"""Heartbeat and message validation independent of MQTT and tkinter."""
from dataclasses import dataclass
import json
import re

def validate_command(command):
    if command in ("START", "STOP"):
        return command
    if re.fullmatch(r"SET_SPEED [0-9]{1,3}", command) and int(command[10:]) <= 100:
        return command
    raise ValueError("Use START, STOP or SET_SPEED 0..100")

@dataclass
class RobotState:
    broker_connected: bool = False
    last_heartbeat: float | None = None
    telemetry: dict | None = None
    result: dict | None = None
    demo: bool | None = None

    def disconnect(self):
        self.broker_connected = False
        self.last_heartbeat = None
        self.telemetry = None
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
            value = json.loads(payload)
        except (ValueError, UnicodeError) as error:
            raise ValueError("Invalid JSON") from error
        if not isinstance(value, dict) or type(value.get("schema")) is not int or value["schema"] != 1 or type(value.get("demo")) is not bool:
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
        elif topic == "command_result":
            if value.get("status") not in ("simulated", "queued", "rejected", "queue_full"):
                raise ValueError("Invalid command result")
            self.result = value
        else:
            raise ValueError("Unknown topic")
        self.demo = value["demo"]
