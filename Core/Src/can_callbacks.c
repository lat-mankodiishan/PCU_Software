#include "can_manager.h"
#include "can.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) {
    if (h == &hcan1) can_mgr_isr_rx_fifo0(CAN_BUS_AVIONICS);
    else if (h == &hcan2) can_mgr_isr_rx_fifo0(CAN_BUS_POWERTRAIN);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h) {
    if (h == &hcan1) can_mgr_isr_tx_complete(CAN_BUS_AVIONICS);
    else if (h == &hcan2) can_mgr_isr_tx_complete(CAN_BUS_POWERTRAIN);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *h) { HAL_CAN_TxMailbox0CompleteCallback(h); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *h) { HAL_CAN_TxMailbox0CompleteCallback(h); }
