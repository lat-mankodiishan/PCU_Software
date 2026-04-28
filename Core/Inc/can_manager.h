#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include "periph_wrappers.h"
#include "cmsis_os2.h"
#include <stdbool.h>

/* Bring up TX queues, RX dispatch task, and arms HW interrupts on both buses. */
void can_mgr_init(void);

/* Enqueue a frame for transmission. timeout_ms = 0 -> non-blocking.
 * Returns true if queued (or immediately handed to a free mailbox). */
bool can_mgr_send(can_bus_t bus, const can_frame_t *f, uint32_t timeout_ms);

/* Subscribe a queue to receive frames matching (id & mask). The caller owns
 * the queue and must drain it. Frames are posted as can_frame_t.
 * Adds a HW filter on the underlying peripheral. Returns true on success. */
bool can_mgr_subscribe(can_bus_t bus,
                       uint32_t   id,
                       uint32_t   mask,
                       bool       ext,
                       osMessageQueueId_t dest_queue);

/* ISR hooks — call from HAL_CAN_Rx*MsgPendingCallback / TxMailboxCompleteCallback. */
void can_mgr_isr_rx_fifo0(can_bus_t bus);
void can_mgr_isr_tx_complete(can_bus_t bus);

/* Per-bus liveness — last RX timestamp (osKernelGetTickCount). 0 = never. */
uint32_t can_mgr_last_rx_tick(can_bus_t bus);

#endif /* CAN_MANAGER_H */
