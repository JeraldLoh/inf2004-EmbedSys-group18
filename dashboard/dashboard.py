"""Run: python dashboard.py --host 192.168.1.10"""
import argparse
import json
import queue
import time
import tkinter as tk
from tkinter import ttk
import paho.mqtt.client as mqtt
from model import RobotState, validate_command, encode_command

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--prefix", default="group18/robot1")
    args = parser.parse_args()
    prefix = args.prefix.rstrip("/")
    root = tk.Tk()
    root.title("Group 18 | Buddy 1")
    root.geometry("620x440")
    frame = ttk.Frame(root, padding=24)
    frame.pack(fill="both", expand=True)
    ttk.Label(frame, text="Buddy 1 • Robot communications", font=("Segoe UI", 17)).pack(anchor="w")
    ttk.Label(frame, text=f"Broker: {args.host}:{args.port} | {prefix}").pack(anchor="w", pady=8)
    state = RobotState()
    events = queue.Queue(maxsize=128)
    def event(*data):
        try: events.put_nowait(data)
        except queue.Full: pass  # Status still expires if callbacks outpace UI.
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.reconnect_delay_set(1, 10)
    client.max_queued_messages_set(8)
    def connected(c, userdata, flags, reason, properties):
        if reason.is_failure:
            event("error", f"Broker rejected connection: {reason}")
            return
        result, _ = c.subscribe([(prefix + "/" + suffix, 0) for suffix in ("telemetry", "heartbeat", "command_result")])
        event("connected", result == mqtt.MQTT_ERR_SUCCESS)
    def disconnected(c, userdata, flags, reason, properties):
        event("disconnected", None)
    def message(c, userdata, msg):
        if not msg.retain and msg.topic.startswith(prefix + "/"):
            event("message", msg.topic[len(prefix)+1:], msg.payload, time.monotonic())
    client.on_connect, client.on_disconnect, client.on_message = connected, disconnected, message
    status = tk.StringVar(value="Connecting…")
    readings = tk.StringVar(value="Waiting for readings")
    mode = tk.StringVar(value="Mode: unknown")
    feedback = tk.StringVar(value="Commands do not confirm physical execution.")
    for variable in (status, mode, readings):
        ttk.Label(frame, textvariable=variable, font=("Segoe UI", 12)).pack(anchor="w", pady=7)
    row = ttk.Frame(frame)
    row.pack(anchor="w", pady=16)
    command_id = 0
    buttons = []
    def send(command):
        nonlocal command_id
        try:
            validate_command(command)
            if state.status(time.monotonic()) != "online" or not client.is_connected():
                raise ValueError("Wait for a live robot heartbeat")
            command_id = command_id % 0xffffffff + 1
            info = client.publish(prefix + "/command", encode_command(command,command_id), qos=0, retain=False)
            feedback.set("Sent to MQTT; waiting for robot result" if info.rc == mqtt.MQTT_ERR_SUCCESS else "Publish failed")
        except ValueError as error: feedback.set(str(error))
    for label, command in (("Start", "START"), ("Stop", "STOP")):
        b = ttk.Button(row, text=label, command=lambda c=command: send(c)); b.pack(side="left", padx=4); buttons.append(b)
    ttk.Label(frame, textvariable=feedback, wraplength=550).pack(anchor="w", pady=8)
    def update():
        for _ in range(128):
            try: item = events.get_nowait()
            except queue.Empty: break
            if item[0] == "connected":
                state.disconnect(); state.broker_connected = item[1]
                if not item[1]: feedback.set("Subscription request failed")
            elif item[0] == "disconnected": state.disconnect()
            elif item[0] == "error": feedback.set(item[1])
            else:
                try:
                    state.receive(item[1], item[2], item[3])
                    if item[1] == "command_result": feedback.set("Robot result: " + state.result["status"] + " (not a physical execution acknowledgement)")
                except ValueError as error: feedback.set("Ignored message: " + str(error))
        # is_connected is also checked so dropped queue events cannot leave controls enabled.
        live = client.is_connected()
        status.set(state.status(time.monotonic()) if live else "broker disconnected")
        mode.set("Mode: " + ("SIMULATED DATA" if state.demo else "robot data" if state.demo is False else "unknown"))
        if state.telemetry:
            age = max(0, time.monotonic() - state.telemetry_at)
            stale = not live or state.status(time.monotonic()) != "online" or age >= 3
            # This optional desktop view is a raw protocol inspector. The web
            # dashboard applies per-module validity and measurement-age rules.
            readings.set(("STALE · " if stale else "") + f"Raw protocol · received {age:.1f}s ago\n"
                         + json.dumps({k: v for k, v in state.telemetry.items() if k != "buddies"}, indent=2)
                         + "\nUse the web dashboard for subsystem validity and age.")
        else:
            readings.set("Waiting for readings")
        enabled = live and state.status(time.monotonic()) == "online"
        for button in buttons: button.configure(state="normal" if enabled else "disabled")
        root.after(100, update)
    def close():
        client.disconnect(); client.loop_stop(); root.destroy()
    root.protocol("WM_DELETE_WINDOW", close)
    client.connect_async(args.host, args.port, keepalive=30)
    client.loop_start()
    update()
    root.mainloop()

if __name__ == "__main__": main()
