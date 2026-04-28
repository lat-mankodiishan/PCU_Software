#ifndef FC_LINK_TASK_H
#define FC_LINK_TASK_H

#include <stdint.h>

/* PCU's DroneCAN node ID. 1..127 valid (1-9 typical for FCs, 50+ for
 * peripherals). Pixhawk FC defaults to 10 — pick anything that doesn't
 * collide. Override at link time if needed. */
#ifndef FC_LINK_NODE_ID
#define FC_LINK_NODE_ID    50u
#endif

/* Create the FC-link task. Call after pt_init(), can_mgr_init(),
 * before osKernelStart(). */
void fc_link_task_start(void);

#endif /* FC_LINK_TASK_H */
