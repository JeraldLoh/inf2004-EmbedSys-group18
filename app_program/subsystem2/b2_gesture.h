#ifndef GROUP18_B2_GESTURE_H
#define GROUP18_B2_GESTURE_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { B2_UI_NONE = 0, B2_UI_SELECTED, B2_UI_CONFIRM, B2_UI_RUN } b2_ui_event_t;
typedef struct { unsigned phase; unsigned selected; uint32_t since; uint32_t held_since; } b2_gesture_t;
b2_ui_event_t b2_gesture_update(b2_gesture_t *gesture, uint32_t now_us,
    bool start_pressed, bool stop_pressed, bool busy);
#endif
