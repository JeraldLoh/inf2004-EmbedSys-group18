#ifndef GROUP18_MOTION_TASK_H
#define GROUP18_MOTION_TASK_H
#include "../app.h"
#include "measured_motion_executor.h"
#include "../vehicle_controller/controller_motion.h"

/* Private driver pointers are provided only by subsystem2's lifecycle. */
ER motion_task_init(const measured_motion_executor_config_t *config,
    const measured_motion_executor_io_t *io, uint32_t boot_session);
ER motion_task_start(void);
/* Bind these callbacks into controller_task before starting either task. */
bool motion_task_outputs(controller_motion_outputs_t *outputs);
bool motion_task_status(measured_motion_executor_status_t *status);
#endif
