#include "b2_gesture.h"
#include <stddef.h>

b2_ui_event_t b2_gesture_update(b2_gesture_t *gesture, uint32_t now,
    bool start, bool stop, bool busy)
{
    if (gesture == NULL) { return B2_UI_NONE; }
    if (stop || busy)
    {
        gesture->phase = 0U;
        gesture->since = now;
        return B2_UI_NONE;
    }
    if (gesture->phase == 0U)
    {
        if (start) { gesture->since = now; }
        else if ((now - gesture->since) >= 30000U) { gesture->phase = 1U; }
    }
    else if (gesture->phase == 1U)
    {
        if (start) { gesture->phase = 2U; gesture->held_since = now; }
    }
    else if (gesture->phase == 2U)
    {
        if (!start)
        {
            gesture->since = now;
            gesture->phase = ((now - gesture->held_since) >= 30000U) ? 5U : 0U;
        }
        else if ((now - gesture->held_since) >= 1000000U)
        {
            gesture->phase = 3U;
            return B2_UI_CONFIRM;
        }
    }
    else if ((now - gesture->held_since) >= 5000000U)
    {
        gesture->phase = 0U;
        gesture->since = now; /* Stale held confirmation cannot run later. */
    }
    else if (gesture->phase == 3U)
    {
        if (!start) { gesture->phase = 4U; gesture->since = now; }
    }
    else if (gesture->phase == 4U)
    {
        if (start) { gesture->phase = 0U; gesture->since = now; }
        else if ((now - gesture->since) >= 30000U)
        {
            gesture->phase = 0U;
            gesture->since = now;
            return B2_UI_RUN;
        }
    }
    else if (gesture->phase == 5U)
    {
        if (start) { gesture->phase = 0U; gesture->since = now; }
        else if ((now - gesture->since) >= 30000U)
        {
            gesture->selected = (gesture->selected + 1U) % 6U;
            gesture->phase = 1U;
            return B2_UI_SELECTED;
        }
    }
    return B2_UI_NONE;
}
