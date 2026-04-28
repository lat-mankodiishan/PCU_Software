/*
 * swo_io.c — retarget stdout to the ITM SWO debug channel.
 *
 * After this file is linked, plain printf("...") shows up in the ST-Link
 * SWV viewer (or any debugger watching ITM channel 0). Channel rate must
 * match the SWV configuration in the debugger:
 *     SWO speed = SYSCLK / (TPI prescaler + 1)
 * For 40 MHz SYSCLK, set the SWV viewer to 40000000 (no prescaler) or
 * 2000000 (prescaler 19) — both work.
 *
 * If the debugger isn't connected, ITM_SendChar() is a near-zero-cost
 * register poll that returns immediately, so leaving printfs in production
 * is acceptable.
 */

#include "stm32f4xx.h"      /* ITM_SendChar */
#include <unistd.h>
#include <sys/types.h>

/* newlib calls _write() from printf/puts/fwrite. Default impl just stubs;
 * we override with ITM. Linker takes our strong symbol over newlib's weak. */
int _write(int file, char *ptr, int len) {
    (void)file;
    for (int i = 0; i < len; ++i) {
        ITM_SendChar((uint32_t)*ptr++);
    }
    return len;
}
