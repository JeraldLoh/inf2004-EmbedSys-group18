#ifndef BUDDY1_BRIDGE_H
#define BUDDY1_BRIDGE_H
#include "buddy1_protocol.h"
#include "buddy1_binary.h"
#define B1_HEARTBEAT 0
#define B1_TELEMETRY 1
#define B1_RESULT 2
#define B1_DIAGNOSTICS 3
#define B1_PAYLOAD_MAX B1_BINARY_MAX
#ifndef B1_DEMO
#define B1_DEMO 1
#endif
/* Plain C ABI: safe to include from the SDK/lwIP namespace or uTKernel. */
typedef struct { unsigned int speed_percent, distance_cm; } b1_snapshot;
typedef struct { unsigned int kind, created_ms, length; char text[B1_PAYLOAD_MAX]; } b1_packet;
int b1_init(void);
void b1_connection(int online);
int b1_network_receive(const char *data, unsigned int n);
int b1_network_take(unsigned int kind, b1_packet *packet);
void b1_network_restore(const b1_packet *packet);
/* Robot-facing API: copies data; returns 1 on success, 0 if busy/full.
   Snapshot producer supplies complete readings. No radio API is exposed. */
int b1_submit_snapshot(const b1_snapshot *snapshot);
/* Complete telemetry snapshot, assembled by an agreed adapter. Absent sections
 * have their present bit clear. Only a producer's owning task copies its state.
 * Returns 0 on invalid input or mutex contention; caller retries later.
 * fault_mask is reserved for agreed hardware/sensor failures; bits not assigned yet. */
int b1_submit_binary_snapshot(const b1_binary_snapshot *snapshot);
int b1_receive_vehicle(b1_command *command);
void b1_telemetry_task(int stacd, void *exinf);
#endif
