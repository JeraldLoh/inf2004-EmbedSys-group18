#ifndef GROUP18_CAR_RUN_TARGET_H
#define GROUP18_CAR_RUN_TARGET_H
#include "../app.h"

/* CAR_RUN=1 only: the first whole-car build (Buddy 5, integration lead).
 *
 * Starts a core-0 supervisor task that brings up, every init before any
 * start: Buddy 2's motor/STOP foundation and draft motion executor, the
 * vehicle controller task (with line tracking), Buddy 3's line/barcode task
 * and the Buddy 5 obstacle task. Nothing moves at boot.
 *
 * Then the supervisor owns the buttons:
 *   GP20 held >= 1 s then released  -> clear operator stops, AUTO: follow line
 *   GP21 STOP (any time)            -> motors off at once (Buddy 2 ISR), latched
 *   cutoff alarm / run time limit   -> stopped; fresh GP20 gesture to restart
 * Motion/controller/sensor faults stay latched until reset.
 *
 * Junction routes are not executed yet: test on a loop without junctions.
 * Not run on hardware. Wheels raised for the first runs.
 */
ER car_run_start(void);

#endif
