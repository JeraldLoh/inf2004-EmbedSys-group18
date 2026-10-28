#ifndef GROUP18_SUBSYSTEM2_H
#define GROUP18_SUBSYSTEM2_H

#include "../app.h"
#include "drv_encoder.h"

/* Startup-only binding, before init and before any task starts. No implicit
 * default port. Init/start stay unavailable until root supplies a valid HAL.
 * No task/PID/goal executor exists in B2-01; start marks the foundation ready
 * without clearing inhibit or starting motors. */
ER subsystem2_bind_hal(const motion_hal_t *hal);
ER subsystem2_shutdown(void);
motion_status_t subsystem2_urgent_stop(void);
motion_status_t subsystem2_read_motor(motion_motor_state_t *state);
motion_status_t subsystem2_read_encoders(motion_encoder_snapshot_t *snapshot);

/* No drive/arm API on the whole-system boundary in this slice. The future B2
 * executor owns drv_motor_apply; controller authority hookup is coordinated. */

#endif
