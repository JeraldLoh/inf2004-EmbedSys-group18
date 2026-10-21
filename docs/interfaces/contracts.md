# Contract Rules

All timestamps and deadlines use a monotonic millisecond clock. Distances use
millimetres, speeds use millimetres per second, and angles use millidegrees.

Every command that can cause movement carries:

- command or goal ID;
- authority/mode generation;
- source;
- creation time and expiry or timeout;
- physical units rather than duty-cycle labels.

Latest-value data, must-handle events, and acquisition buffers are separate
communication classes. A future micro T-Kernel adapter must preserve these
semantics using only primitives supported by the verified course port.

The official course source is now imported. Existing portable declarations are
in `app_program/contracts/`; the controller is in `app_program/vehicle_controller/`.
The C types do not allocate kernel objects or make concurrent access safe.

This migration preserves prior logic instead of pretending it completes the
contracts. Before runtime use, review missing finite-goal creation/freshness
metadata, sensor freshness, route/junction association, recovery permits, stop
clear/re-arm authorization, line-stability completion and overflow policies.
Existing limited host checks do not establish complete safety behavior.

Raw network calls belong to the built-in service task; Buddy 1 needs a reviewed
bounded bridge. SMP-safe ownership, copy/lifetime rules and exact ISR-safe
signalling primitives must be agreed in TEAM.md before adapter implementation.

Pointers to temporary stack objects must never be placed in asynchronous
messages. Queue-full handling must be explicit. Urgent stop delivery must not
depend on an ordinary queue having free capacity.

## Barcode lifecycle

```text
NONE -> VALIDATED -> PENDING -> RESERVED -> CONSUMED
                    |             |
                    +-> EXPIRED   +-> CANCELLED
```

A-D map to left, right, straight, and U-turn. Other valid Code 39 characters
have no movement authority unless the shared specification is updated.
