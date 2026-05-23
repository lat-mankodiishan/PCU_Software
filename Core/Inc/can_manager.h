#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "periph_wrappers.h"
#include "cmsis_os2.h"
#include <stdbool.h>

void can_mgr_init(void);

/* timeout_ms=0 non-blocking; true if queued/handed to mailbox. */
bool can_mgr_send(can_bus_t bus, const can_frame_t *f, uint32_t timeout_ms);

/* Subscribe queue to frames matching (id & mask); adds HW filter. */
bool can_mgr_subscribe(can_bus_t bus,
                       uint32_t   id,
                       uint32_t   mask,
                       bool       ext,
                       osMessageQueueId_t dest_queue);

/* Call from HAL_CAN_Rx*MsgPending / TxMailboxComplete callbacks. */
void can_mgr_isr_rx_fifo0(can_bus_t bus);
void can_mgr_isr_tx_complete(can_bus_t bus);

/* Last RX tick (0 = never). */
uint32_t can_mgr_last_rx_tick(can_bus_t bus);

#endif /* CAN_MANAGER_H */
