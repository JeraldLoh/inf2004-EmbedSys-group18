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
