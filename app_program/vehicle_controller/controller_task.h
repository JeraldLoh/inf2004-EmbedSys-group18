#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

/* Kernel wrapper around controller_service: one task owns the vehicle
 * controller and applies events from a message buffer, so no two tasks ever
 * change controller state at the same time. Between events it runs
 * controller_service_tick() every 10 ms (line tracking, when wired).
 *
 * Lifecycle: controller_task_init() with every other init, before any task
 * starts; controller_task_start() afterwards. Task priority and stack are
 * provisional until measured with the other team tasks.
 */
#include "../app.h"
#include "controller_service.h"
#include "controller_motion.h"

ER controller_task_init(const controller_outputs_t *outputs);
ER controller_task_start(void);

/* From any task (not an ISR). Never blocks. Ordinary events use FIFO and
 * reject a full queue; STOP uses an independent sticky lane and disables
 * outputs immediately even when FIFO is full. */
bool controller_post(const controller_event_t *event);

/* Copy of the latest status; false if not initialised or busy. */
bool controller_get_status(controller_status_t *status);
/* Junctions (see controller_service.h): after controller_task_init() with
 * the line-tracking outputs, before any start. Not with the typed motion
 * adapter below. */
ER controller_task_bind_junction(const controller_junction_outputs_t *outputs);
/* Bind after controller init and before any start. Existing STOP-only users
 * need no change. No hook means motion commands fail closed. */
ER controller_task_bind_motion(const controller_motion_outputs_t *outputs,
    uint32_t boot_session, uint32_t caution_mm_s, uint32_t cruise_mm_s);
bool controller_post_motion(const controller_motion_request_t *request);
bool controller_post_motion_result(const motion_goal_result_t *result);
bool controller_post_motion_stop(stop_reason_mask_t reasons, uint32_t evidence_id);
bool controller_get_motion_status(controller_motion_status_t *status);
bool controller_urgent_stop_pending(void);

/* May source drive now? See controller_service_authority(). Any task. */
bool controller_authority(motion_source_t source, authority_generation_t *generation);

#endif
