#include "subsystem2.h"
#include "b2_platform_rp2040.h"
#include "motion_hal_rp2040.h"
#include "../project_config/pinmap.h"
#include <tk/smp.h>
#include <bsp/libbsp.h>
#include <sys/sysdef.h>

/* Offsets verified against SDK 2.3.1 RP2040 hardware/regs/io_bank0.h.
 * No SDK runtime/callback owner is installed. Only core 0 enables GPIO IRQ. */
#define B2_GPIO_INTR(n) (IO_BANK0_BASE + 0xF0U + (4U * (n)))
#define B2_GPIO_INTE(n) (IO_BANK0_BASE + 0x100U + (4U * (n)))
#define B2_GPIO_INTS(n) (IO_BANK0_BASE + 0x120U + (4U * (n)))
#define B2_GPIO_CORE1_INTE(n) (IO_BANK0_BASE + 0x130U + (4U * (n)))
#define B2_GPIO_IRQ 13U
#define B2_TIMER_IRQ 0U
#define B2_ENCODERS (((uint32_t)1U << PIN_ENCODER_LEFT) | ((uint32_t)1U << PIN_ENCODER_RIGHT))
#define B2_STOP_PIN ((uint32_t)1U << PIN_BUTTON_STOP)

#if TK_SUPPORT_SMP
static T_SPLOCK motion_lock;
static gpio_dispatcher_t dispatcher;
static motion_rp2040_port_t port;
static b2_platform_events_t evidence;
static uint32_t entered_us;
static bool initialized;

uint32_t b2_platform_now_us(void) { return (uint32_t)in_w(TIMER_TIMERAWL); }

static uintptr_t enter(void *context)
{
    UINT saved;
    uint32_t before = b2_platform_now_us();
    uint32_t wait;
    (void)context;
    /* Valid static aligned lock/status addresses: ISpinLock only fails E_PAR
     * for NULL pointers. It masks local IRQs BEFORE spinning, uses kernel's
     * atomic lock word, and acquires no big kernel lock. No other kernel API,
     * callback or console operation is called while motion_lock is held. */
    (void)ISpinLock(&motion_lock, &saved);
    entered_us = b2_platform_now_us();
    wait = entered_us - before;
    if (wait > evidence.max_lock_wait_us) { evidence.max_lock_wait_us = wait; }
    return saved;
}

static void leave(void *context, uintptr_t key)
{
    uint32_t hold = b2_platform_now_us() - entered_us;
    (void)context;
    if (hold > evidence.max_lock_hold_us) { evidence.max_lock_hold_us = hold; }
    (void)ISpinUnlock(&motion_lock, (UINT)key);
}

static bool registration_allowed(void *context)
{
    (void)context;
    return (in_w(SIO_CPUID_ADDR) == 0U) && (in_w(SCB_ICSR) % 512U == 0U);
}

static uint32_t edge_fields(uint32_t pins, unsigned group, unsigned nibble)
{
    unsigned bit;
    uint32_t fields = 0U;
    for (bit = 0U; bit < 8U; ++bit)
    {
        unsigned pin = (8U * group) + bit;
        if ((pin < 30U) && ((pins & ((uint32_t)1U << pin)) != 0U))
        {
            fields |= (uint32_t)nibble << (4U * bit);
        }
    }
    return fields;
}

static bool enable_edges(void *context, uint32_t pins, bool enabled)
{
    unsigned group;
    (void)context;
    for (group = 0U; group < 4U; ++group)
    {
        uint32_t fields = edge_fields(pins, group, 12U);
        uint32_t old = (uint32_t)in_w(B2_GPIO_INTE(group));
        out_w(B2_GPIO_INTE(group), enabled ? old | fields : old & ~fields);
    }
    return true;
}

static void ack_edges(void *context, uint32_t rising, uint32_t falling)
{
    unsigned group;
    (void)context;
    for (group = 0U; group < 4U; ++group)
    {
        out_w(B2_GPIO_INTR(group), edge_fields(rising, group, 8U) |
            edge_fields(falling, group, 4U));
    }
}

static bool clock_us(void *context, uint32_t *now)
{
    (void)context;
    *now = b2_platform_now_us();
    return true;
}

static bool attach_encoders(void *context, motion_edge_handler_t handler, void *user)
{
    (void)context;
    return gpio_dispatcher_attach(&dispatcher, B2_ENCODERS, handler, user, false);
}

static bool detach_encoders(void *context)
{
    (void)context;
    return gpio_dispatcher_detach(&dispatcher, B2_ENCODERS);
}

bool b2_platform_stop_pressed(void) { return gpio_get_val(PIN_BUTTON_STOP) == 0U; }
bool b2_platform_start_pressed(void) { return gpio_get_val(PIN_BUTTON_START) == 0U; }

static void confirmed_stop(stop_reason_mask_t reasons, bool cutoff)
{
    uint32_t confirmed = b2_platform_now_us();
    motion_status_t stopped;
    uintptr_t key = enter(NULL);
    evidence.reasons |= reasons;
    evidence.cutoff_fired |= cutoff;
    evidence.confirmed_us = confirmed;
    if (evidence.stop_events != UINT32_MAX) { ++evidence.stop_events; }
    leave(NULL, key);
    stopped = subsystem2_urgent_stop();
    key = enter(NULL);
    if (stopped != MOTION_OK) { evidence.reasons |= STOP_REASON_MOTION_FAULT; }
    evidence.output_command_us = b2_platform_now_us();
    leave(NULL, key);
}

static void physical_stop(void *user, uint32_t rising, uint32_t falling, uint32_t when)
{
    (void)user; (void)rising; (void)when;
    /* Assert at FIRST falling edge, no blocking debounce. Bounce may assert
     * extra STOPs but can never clear inhibit. Task checks held level too. */
    if ((falling & B2_STOP_PIN) != 0U) { confirmed_stop(STOP_REASON_PHYSICAL, false); }
}

static void gpio_irq(UINT intno)
{
    unsigned group;
    unsigned bit;
    uint32_t rising = 0U;
    uint32_t falling = 0U;
    uint32_t when = b2_platform_now_us();
    (void)intno;
    for (group = 0U; group < 4U; ++group)
    {
        uint32_t fields = (uint32_t)in_w(B2_GPIO_INTS(group));
        for (bit = 0U; bit < 8U; ++bit)
        {
            unsigned pin = (8U * group) + bit;
            if (pin < 30U)
            {
                if ((fields & ((uint32_t)8U << (4U * bit))) != 0U)
                { rising |= (uint32_t)1U << pin; }
                if ((fields & ((uint32_t)4U << (4U * bit))) != 0U)
                { falling |= (uint32_t)1U << pin; }
            }
        }
    }
    gpio_dispatcher_dispatch(&dispatcher, rising, falling, when);
}

static void timer_irq(UINT intno)
{
    (void)intno;
    out_w(TIMER_INTR, 1U);
    out_w(TIMER_INTE, in_w(TIMER_INTE) & ~1U);
    confirmed_stop(STOP_REASON_NONE, true);
}

static bool stop_pressed(void *context) { (void)context; return b2_platform_stop_pressed(); }
static bool stop_pending(void *context)
{
    bool pending;
    uintptr_t key = enter(context);
    pending = (evidence.reasons != STOP_REASON_NONE) || evidence.cutoff_fired;
    leave(context, key);
    return pending;
}

static bool arm_cutoff(void *context, uint32_t duration_us)
{
    bool ok;
    uint32_t deadline;
    uintptr_t key = enter(context);
    ok = initialized && (duration_us > 0U) && (duration_us <= B2_TRIAL_MAX_US) &&
        !b2_platform_stop_pressed() && (evidence.reasons == STOP_REASON_NONE) &&
        !evidence.cutoff_fired;
    if (ok)
    {
        out_w(TIMER_INTR, 1U);
        deadline = b2_platform_now_us() + duration_us;
        out_w(TIMER_ALARM0, deadline);
        out_w(TIMER_INTE, in_w(TIMER_INTE) | 1U);
        if ((b2_platform_now_us() - deadline) < 0x80000000UL)
        {
            /* Never enable a trial if a tiny/late alarm deadline was missed
             * during setup; otherwise equality could wait a whole timer wrap. */
            out_w(TIMER_INTE, in_w(TIMER_INTE) & ~1U);
            out_w(TIMER_ARMED, 1U);
            out_w(TIMER_INTR, 1U);
            ok = false;
        }
    }
    leave(context, key);
    return ok;
}

static void cancel_cutoff(void *context)
{
    uintptr_t key = enter(context);
    out_w(TIMER_INTE, in_w(TIMER_INTE) & ~1U);
    out_w(TIMER_ARMED, 1U);
    out_w(TIMER_INTR, 1U);
    leave(context, key);
}

bool b2_platform_init(motion_hal_t *hal, b2_trial_io_t *trial_io)
{
#if TK_SUPPORT_SMP
    unsigned group;
    unsigned pin;
    unsigned probe;
    uint32_t clock_before;
    bool safe = true;
    uintptr_t key;
    T_DINT dint = {0};
    gpio_dispatch_io_t io = {NULL, enter, leave, registration_allowed,
        enable_edges, ack_edges};
    if ((hal == NULL) || (trial_io == NULL) || initialized ||
        !registration_allowed(NULL)) { return false; }
    if (InitSpinLock(&motion_lock) != E_OK) { return false; }
    key = enter(NULL);
    /* Bootstrap owns motor pins before any HAL client/IRQ exists. Ensure a
     * disabled command even if later IRQ setup/registration is refused. */
    for (pin = PIN_MOTOR_LEFT_A; pin <= PIN_MOTOR_RIGHT_B; ++pin)
    {
        if (pwm_set_cc(pin, 0U) != E_OK) { safe = false; }
        if (gpio_set_val(pin, 0U) != E_OK) { safe = false; }
        if (gpio_set_pin(pin, GPIO_MODE_OUT) != E_OK) { safe = false; }
    }
    if (pwm_set_enabled(PIN_MOTOR_LEFT_A, FALSE) != E_OK) { safe = false; }
    if (pwm_set_enabled(PIN_MOTOR_RIGHT_A, FALSE) != E_OK) { safe = false; }
    leave(NULL, key);
    if (!safe || ((in_w(TIMER_INTE) & 1U) != 0U) ||
        ((in_w(TIMER_ARMED) & 1U) != 0U)) { return false; }
    /* No unbounded wait and no global timer/tick reconfiguration. The ROM /
     * kernel clock must already be live before a bounded motion is admitted. */
    clock_before = b2_platform_now_us();
    for (probe = 0U; probe < 10000U; ++probe)
    {
        if (b2_platform_now_us() != clock_before) { break; }
    }
    if (probe == 10000U) { return false; }
    for (group = 0U; group < 4U; ++group)
    {
        if ((in_w(B2_GPIO_INTE(group)) != 0U) ||
            (in_w(B2_GPIO_CORE1_INTE(group)) != 0U)) { return false; }
    }
    if (!gpio_dispatcher_init(&dispatcher, &io)) { return false; }
    DisableInt(B2_GPIO_IRQ);
    DisableInt(B2_TIMER_IRQ);
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_irq;
    if (tk_def_int(B2_GPIO_IRQ, &dint) != E_OK) { return false; }
    dint.inthdr = (FP)timer_irq;
    if (tk_def_int(B2_TIMER_IRQ, &dint) != E_OK)
    {
        (void)tk_def_int(B2_GPIO_IRQ, NULL);
        return false;
    }
    key = enter(NULL);
    if ((gpio_set_pin(PIN_BUTTON_STOP, GPIO_MODE_IN) != E_OK) ||
        (gpio_set_pin(PIN_BUTTON_START, GPIO_MODE_IN) != E_OK))
    {
        leave(NULL, key);
        return false;
    }
    set_w(GPIO(PIN_BUTTON_STOP), GPIO_PUE);
    clr_w(GPIO(PIN_BUTTON_STOP), GPIO_PDE);
    set_w(GPIO(PIN_BUTTON_START), GPIO_PUE);
    clr_w(GPIO(PIN_BUTTON_START), GPIO_PDE);
    leave(NULL, key);
    if (!gpio_dispatcher_attach(&dispatcher, B2_STOP_PIN, physical_stop, NULL, true))
    { return false; }
    port = (motion_rp2040_port_t){NULL, enter, leave, clock_us,
        attach_encoders, detach_encoders};
    if (!motion_hal_rp2040_create(hal, &port)) { return false; }
    *trial_io = (b2_trial_io_t){NULL, stop_pressed, stop_pending, arm_cutoff, cancel_cutoff};
    initialized = true;
    ClearInt(B2_GPIO_IRQ);
    ClearInt(B2_TIMER_IRQ);
    EnableInt(B2_GPIO_IRQ, 0);
    EnableInt(B2_TIMER_IRQ, 0);
    return true;
#else
    (void)hal; (void)trial_io;
    return false; /* Bench build requires SMP=1; no fake cross-core guarantee. */
#endif
}

void b2_platform_take_events(b2_platform_events_t *events)
{
    uintptr_t key;
    if ((events == NULL) || !initialized) { return; }
    key = enter(NULL);
    *events = evidence;
    evidence.reasons = STOP_REASON_NONE;
    evidence.cutoff_fired = false;
    leave(NULL, key);
}

gpio_dispatcher_t *b2_platform_dispatcher(void) { return &dispatcher; }
#else
bool b2_platform_init(motion_hal_t *hal, b2_trial_io_t *io)
{ (void)hal; (void)io; return false; }
void b2_platform_take_events(b2_platform_events_t *events)
{ if (events != NULL) { *events = (b2_platform_events_t){0}; } }
gpio_dispatcher_t *b2_platform_dispatcher(void) { return NULL; }
bool b2_platform_start_pressed(void) { return false; }
bool b2_platform_stop_pressed(void) { return true; }
uint32_t b2_platform_now_us(void) { return (uint32_t)in_w(TIMER_TIMERAWL); }
#endif
