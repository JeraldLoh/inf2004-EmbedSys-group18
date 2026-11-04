# Restore Buddy 3 ownership - 2 November 2026

The user requests that Buddy 3 match testing before the Buddy 2 publication,
and that future implementation edits be confined to Buddy 2.

Restored all app_program/subsystem3 files exactly to testing641ad74. No later
teammate changes to that directory exist in the fetched testing history.
This removes the trial curve memory, its fields/tuning-header dependency,
and the raw-barcode API extension introduced for circuit diagnostics.

Restored the pre-publication CAR_RUN steering build flags for normal and
trial images: ramp6ms, hold20, damping0, with original Buddy3 defaults for
steering base/max and sensor polarity. Removed trial LINE_* overrides.
The Buddy3 curve-memory test and its registration are removed as superseded.
The circuit diagnostic no longer calls the reverted barcode API; its raw ADC
display is explicitly unavailable rather than adding a second ADC reader.
The trial banner no longer advertises removed steering/memory settings.
These support edits undo dependencies/overrides from our Buddy3 changes.

The publication is based on testing67ae87a and preserves its Buddy5 servo/
avoidance changes. Buddy2's previously published motor/encoder/PID work remains.
Unpublished V10/V11 turn experiments are not included in this restoration push.
Main and Buddy-2 branches are not updated by this testing correction.

Earlier logs and guides describe historical firmware and must not be read as
current steering settings. Known controller resume/line-loss concerns remain;
they are not fixed as part of this ownership restoration. No automated tests,
flash or vehicle movement were performed. Existing UF2s contain older source;
the source rollback alone does not change an already flashed Pico.
