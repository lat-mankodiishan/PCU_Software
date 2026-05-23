#ifndef FC_LINK_TASK_H
#define FC_LINK_TASK_H

#include <stdint.h>

/* PCU DroneCAN node ID; Pixhawk defaults to 10. */
#ifndef FC_LINK_NODE_ID
#define FC_LINK_NODE_ID    50u
#endif

/* Call after pt_init(), can_mgr_init(), before osKernelStart(). */
void fc_link_task_start(void);

#endif /* FC_LINK_TASK_H */
