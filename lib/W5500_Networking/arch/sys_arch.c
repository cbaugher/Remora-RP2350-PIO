/* sys_arch.c — RP2350 port of the LwIP sys_arch stub.
 *
 * sys_now() is the only function required by LwIP in NO_SYS mode.
 * It must return the current time in milliseconds and must never
 * wrap within a ~50-day window (uint32_t overflow is fine for LwIP).
 *
 * to_ms_since_boot(get_absolute_time()) is the pico-sdk equivalent
 * of HAL_GetTick() / uwTick on STM32.
 */

#include "pico/stdlib.h"
#include "pico/time.h"

/* Returns the current time in mS. This is needed for the LWIP timers */
uint32_t sys_now(void)
{
    return to_ms_since_boot(get_absolute_time());
}
