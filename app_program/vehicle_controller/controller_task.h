#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

/* Kernel wrapper around controller_service: one task owns the vehicle
 * controller and applies events from a message buffer, so no two tasks ever
 * change controller state at the same time.
 *
 * Lifecycle: controller_task_init() with every other init, before any task
 * starts; controller_task_start() afterwards. Task priority and stack are
 * provisional until measured with the other team tasks.
 */
#include "../app.h"
#include "controller_service.h"

ER controller_task_init(const controller_outputs_t *outputs);
ER controller_task_start(void);

/* From any task (not an ISR). Never blocks; false if not started or the
 * queue is full. Events are applied in the order they were posted. */
bool controller_post(const controller_event_t *event);

/* Copy of the latest status; false if not initialised or busy. */
bool controller_get_status(controller_status_t *status);

#endif
