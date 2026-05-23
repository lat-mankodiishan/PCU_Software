/* swo_io — retarget stdout to ITM SWO channel 0. */

#include "stm32f4xx.h"
#include <unistd.h>
#include <sys/types.h>

/* Strong override of newlib weak _write -> ITM. */
int _write(int file, char *ptr, int len) {
    (void)file;
    for (int i = 0; i < len; ++i) {
        ITM_SendChar((uint32_t)*ptr++);
    }
    return len;
}
