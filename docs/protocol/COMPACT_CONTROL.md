# Compact MQTT control messages

The compact-data firmware publishes two outgoing binary types on the existing topics under `group18/robot1`. Integers are little-endian. The web and Python dashboards accept these types and the legacy JSON heartbeat and command result used by the 180° rejoin UF2. Commands sent to the rover keep the existing binary format.

| Topic | Offsets and fields | Payload |
|---|---|---:|
| `/heartbeat` | Bytes 0–2 are `B1 01 05`. Byte 3 bit 0 is demo and the other bits are zero. Bytes 4–7 are uint32 sequence. Bytes 8–11 are uint32 uptime in ms. | 12 B |
| `/command_result` | Bytes 0–2 are `B1 01 06`. Byte 3 holds status in bits 0–2, demo in bit 3, map present in bit 4. Bits 5–7 are zero. Bytes 4–7 hold the uint32 command ID. | 8 B |
| `/command_result` with an applied map | Common header followed by uint32 revision in bytes 8–11 and the existing 14-byte packed map in bytes 12–25. | 26 B |

Result status values are 0 simulated, 1 queued, 2 rejected, 3 queue full and 4 applied. Applied results carry a map. The command ID must be nonzero for an applied map. An undecodable command can return rejection with ID zero. Map revisions are nonzero and even. Map entries are 3-bit values 0–4, ordered A–Z then 0–9. Each assigned action may occur once. The four high padding bits of the final byte are zero.

Cross-language test vector for demo heartbeat sequence `0xffffffff` and uptime `0x12345678` is `b1010501ffffffff78563412`. An empty applied map with command ID `0xffffffff` and revision `0xfffffffe` is `b1010614fffffffffeffffff` followed by 14 zero bytes.

The heartbeat retains its 5-second generation cadence and the web dashboard's 15-second timeout. A retained heartbeat leaves web dashboard status unchanged. A malformed binary result leaves the last valid route map in place. The `/log` topic continues to carry readable diagnostic text for export.
