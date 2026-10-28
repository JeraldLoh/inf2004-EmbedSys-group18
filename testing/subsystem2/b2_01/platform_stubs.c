#include "platform_stubs.h"
#include <tk/smp.h>
#include <sys/sysdef.h>
#include <assert.h>

uint32_t mock_core;
uint32_t mock_exception;
uint32_t mock_timer_alarm;
uint32_t mock_timer_enabled;
bool mock_stop_during_arm;
uint32_t mock_alarm_setup_delay;
bool mock_gpio_owned;
unsigned mock_fail_irq = 32U;
bool mock_clock_step = true;
unsigned mock_lock_entries;
static uint32_t gpio_raw[4];
static uint32_t gpio_enabled[4];
static uint32_t pads[30];
static FP handlers[32];
static bool irq_enabled[32];
static UINT primask;
static bool pending_stop;

UW in_w(UW address)
{
    if (address == SIO_CPUID_ADDR) { return mock_core; }
    if (address == SCB_ICSR) { return mock_exception; }
    if (address == TIMER_TIMERAWL)
    { if (mock_clock_step) { ++bsp_fake->now_us; } return bsp_fake->now_us; }
    if (address == TIMER_INTE) { return mock_timer_enabled; }
    if (address == TIMER_ARMED) { return 0U; }
    if ((address >= IO_BANK0_BASE + 0x130U) && (address < IO_BANK0_BASE + 0x140U))
    { return 0U; }
    if ((address >= IO_BANK0_BASE + 0x100U) && (address < IO_BANK0_BASE + 0x110U))
    { return mock_gpio_owned ? 1U : gpio_enabled[(address - IO_BANK0_BASE - 0x100U) / 4U]; }
    if ((address >= IO_BANK0_BASE + 0x120U) && (address < IO_BANK0_BASE + 0x130U))
    {
        unsigned group = (address - IO_BANK0_BASE - 0x120U) / 4U;
        return gpio_raw[group] & gpio_enabled[group];
    }
    if ((address >= GPIO(0)) && (address <= GPIO(29)))
    { return pads[(address - GPIO(0)) / 4U]; }
    assert(false);
    return 0U;
}

void out_w(UW address, UW value)
{
    assert(bsp_fake->locked || (mock_exception != 0U));
    if (address == TIMER_INTE) { mock_timer_enabled = value; return; }
    if (address == TIMER_ALARM0)
    {
        mock_timer_alarm = value;
        bsp_fake->now_us += mock_alarm_setup_delay;
        if (mock_stop_during_arm) { pending_stop = true; mock_stop_during_arm = false; }
        return;
    }
    if ((address == TIMER_INTR) || (address == TIMER_ARMED)) { return; }
    if ((address >= IO_BANK0_BASE + 0x100U) && (address < IO_BANK0_BASE + 0x110U))
    { gpio_enabled[(address - IO_BANK0_BASE - 0x100U) / 4U] = value; return; }
    if ((address >= IO_BANK0_BASE + 0xF0U) && (address < IO_BANK0_BASE + 0x100U))
    { gpio_raw[(address - IO_BANK0_BASE - 0xF0U) / 4U] &= ~value; return; }
    if ((address >= GPIO(0)) && (address <= GPIO(29)))
    { pads[(address - GPIO(0)) / 4U] = value; return; }
    assert(false);
}

ER InitSpinLock(T_SPLOCK *lock) { *lock = 0U; return E_OK; }
ER ISpinLock(T_SPLOCK *lock, UINT *saved)
{
    assert(!bsp_fake->locked && (*lock == 0U));
    *saved = primask;
    primask = 1U;
    *lock = 1U;
    bsp_fake->locked = true;
    ++mock_lock_entries;
    return E_OK;
}
ER ISpinUnlock(T_SPLOCK *lock, UINT saved)
{
    assert(bsp_fake->locked && (primask == 1U) && (*lock == 1U));
    *lock = 0U;
    bsp_fake->locked = false;
    primask = saved;
    if (pending_stop)
    {
        pending_stop = false;
        bsp_fake->inputs[21] = false;
        mock_gpio_edges(0U, (uint32_t)1U << 21);
        mock_fire_irq(13U);
    }
    return E_OK;
}
ER tk_def_int(UINT irq, const T_DINT *handler)
{
    assert(!bsp_fake->locked && (irq < 32U));
    if ((irq == mock_fail_irq) && (handler != NULL)) { return E_SYS; }
    handlers[irq] = (handler != NULL) ? handler->inthdr : NULL;
    return E_OK;
}
void DisableInt(UINT irq) { irq_enabled[irq] = false; }
void EnableInt(UINT irq, int priority)
{ assert(priority == 0); irq_enabled[irq] = true; }
void ClearInt(UINT irq) { (void)irq; }
void mock_fire_irq(unsigned irq)
{
    assert((irq < 32U) && irq_enabled[irq] && (handlers[irq] != NULL));
    assert(!bsp_fake->locked && (primask == 0U));
    mock_exception = 16U + irq;
    handlers[irq](irq);
    mock_exception = 0U;
}
void mock_gpio_edges(uint32_t rising, uint32_t falling)
{
    unsigned pin;
    for (pin = 0U; pin < 30U; ++pin)
    {
        if ((rising & ((uint32_t)1U << pin)) != 0U)
        { gpio_raw[pin / 8U] |= (uint32_t)8U << (4U * (pin % 8U)); }
        if ((falling & ((uint32_t)1U << pin)) != 0U)
        { gpio_raw[pin / 8U] |= (uint32_t)4U << (4U * (pin % 8U)); }
    }
}
