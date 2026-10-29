# Static RAM build comparison — 28 October 2026

Matched compiler, SDK 2.3.0, linker configuration and demo build flags:

`NET=lwip MQTT=1 TEST=1 B1_DEMO_RUN=1 CONSOLE=usb_cdc CRASH_REPORT=0 SMP=1`

| Metric | Text baseline (`4798953`) | Binary + diagnostics prototype | Change |
|---|---:|---:|---:|
| .data | 96 B | 96 B | +0 B |
| .bss | 82784 B | 84352 B | +1568 B |
| **Static RAM (.data + .bss)** | **82880 B** | **84448 B** | **+1568 B** |
| .text (flash, not RAM) | 333952 B | 343740 B | +9788 B |

Values were read from the linked ELF files using `arm-none-eabi-size`, not
estimated from payload size. ELF and relevant source hashes, compiler version and
flags are retained in `dashboard/web/build-memory.json`. Placeholder credentials
were used in both builds; no board was flashed.

This records **an increase**, not a RAM saving: the current prototype adds Buddy
3–5 schemas, larger transport buffers and diagnostics to the two-field text
baseline. It does not isolate binary packing alone or prove minimum-bit storage
throughout the project. A same-feature text/binary build comparison is separate.

These are historical build values. They do not measure current task stack usage,
heap consumption, high-water marks or the firmware actually flashed on the car.
Reserved heap backing memory may already be included in .bss. Regenerate this
evidence after firmware changes before treating it as current.

Telemetry's full current wire proposal is 252 bytes including Buddy 2; diagnostics adds a
separate 52-byte payload approximately once per second, excluding MQTT overhead.
The dashboard computes an equivalent JSON payload size from the same represented
values, without duplicating Buddy 4's derived display fields. This is an encoding
comparison, not a captured historical packet or a RAM measurement.
