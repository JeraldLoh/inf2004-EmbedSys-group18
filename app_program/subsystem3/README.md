# Buddy 3 - Line, Barcode, and Junction Perception

Primary responsibility: IR acquisition, line evidence, steering proposals,
barcode decoding, and junction observations.

This subsystem publishes evidence and never writes motor PWM. Other buddies
may contribute, but Buddy 3 remains accountable for its contracts, calibration,
and evidence.

The course folder is `subsystem3`. `barcode_decoder.c/.h` ports the reusable,
hardware-independent Code 39 core and maps only complete `*A*`-`*D*` symbols to
the shared route-command enum. It still needs recorded sample/vehicle evidence.
Place future `drv_ir.c/.h` and acquisition files directly here. Lifecycle and
target-test files are inactive
placeholders, not drivers or successful tests. Use GP7 (left) and GP0 (right)
for digital line IR and GP26/ADC0 for barcode only after G1 verification. To
use the barcode input, set `DEVCONF_ADC_PIN_INIT_0` in
`device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`; leave channel 1 off, because
GP27 is Buddy 2's left encoder.

UART console and `CRASH_REPORT=1` use GP0 (UART0 TX) and conflict with the
right line sensor.
The planned car profile is USB console with crash mirroring disabled. Mounting
height, contrast, junction observability and sampling performance require tests.
Save evidence in `../../testing/subsystem3/`.
