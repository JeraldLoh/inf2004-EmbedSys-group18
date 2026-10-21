# Application platform integration

The course source template is now inspected and imported. Read
`../../docs/platform/course-template-review.md` for the verified startup, build,
network-owner and SMP constraints. This is no longer blocked by a missing URL.

Actual adapters remain unimplemented. Confirm supported ISR calls and locking,
interrupt routing/reservations, timeout units, monotonic time and task ownership
before writing them. Do not copy the standalone Pico SDK callback/startup pattern
or a local interrupt-disable lock into this dual-core kernel without review.

Keep application adapters here, drivers in their owning `subsystemN`, and kernel
support in the imported course locations. Changes to shared library code require
explicit review, not an unrecorded workaround.
