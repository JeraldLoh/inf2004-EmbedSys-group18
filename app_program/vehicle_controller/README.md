# Shared Vehicle Controller

All five buddies may contribute; Buddy 5 leads integration review. This is NOT
Buddy 5's obstacle module and is NOT a sixth student's subsystem.

The portable `vehicle_controller.c/.h` and `safety.c/.h` were moved unchanged from
the old `firmware/app` location. They are partial, host-testable state logic, not
a complete controller or a working RTOS task. Existing limitations were not fixed
by this structural migration.

A dedicated controller task, kernel message transport, button/remote adapters,
goal dispatch, line-stability confirmation, recovery permits and full fault policy
are future work. Keep a single runtime owner of mutable controller state. Do not
call the portable state-mutating functions concurrently from both cores.

Future lifecycle integration must create controller AND subsystem objects before
starting any application task. This directory is directly under `app_program`
so its `.c` files are discovered by the inspected one-level course Make rule.
