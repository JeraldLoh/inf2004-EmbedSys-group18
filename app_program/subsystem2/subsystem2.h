#ifndef GROUP18_SUBSYSTEM2_H
#define GROUP18_SUBSYSTEM2_H

#include "../app.h"
#include "drv_encoder.h"

/* Startup-only binding, before init and before any task starts. No implicit
 * default port. Init/start stay unavailable until root supplies a valid HAL.
 * Foundation start does not clear inhibit or start motors. */
ER subsystem2_bind_hal(const motion_hal_t *hal);
ER subsystem2_shutdown(void);
motion_status_t subsystem2_urgent_stop(void);
motion_status_t subsystem2_read_motor(motion_motor_state_t *state);
motion_status_t subsystem2_read_encoders(motion_encoder_snapshot_t *snapshot);

/* Startup-only access for the B2 task adapter, unavailable in raw bench builds.
 * These pointers must never be used by other subsystems to apply wheel power. */
bool subsystem2_executor_drivers(drv_motor_t **motor, drv_encoder_t **encoder);

#endif
