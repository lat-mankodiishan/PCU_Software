#ifndef PERIPH_WRAPPERS_H
#define PERIPH_WRAPPERS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CAN_BUS_DRONECAN = 0,   /* CAN1 PCU<->FC */
    CAN_BUS_ENGINE   = 1,   /* CAN2 Rectifier+ECU+Laptop */
    CAN_BUS_COUNT
} can_bus_t;

typedef struct {
    uint32_t id;            /* 11- or 29-bit */
    uint8_t  ext : 1;
    uint8_t  rtr : 1;
    uint8_t  dlc : 4;       /* 0..8 */
    uint8_t  data[8];
} can_frame_t;

/* Call after MX_CANx_Init. */
void can_hw_init(can_bus_t bus);

/* Non-blocking; true if mailbox accepted. */
bool can_hw_try_post(can_bus_t bus, const can_frame_t *f);

uint8_t can_hw_tx_free(can_bus_t bus);

/* Returns filter bank index, or -1 on no slots. */
int can_hw_add_filter(can_bus_t bus, uint32_t id, uint32_t mask, bool ext);

bool can_hw_read_fifo0(can_bus_t bus, can_frame_t *out);

/* true = closed. */
void pdb_hw_write(bool battery_closed, bool rectifier_closed);

void watchdog_refresh(void);
void led_hw_toggle(void);

/* DEBUG: CAN1 ESR/TEC/REC into globals. */
void can_hw_diag_snapshot(void);

/* True while bxCAN reports bus-off (ESR.BOFF). Auto-recovery is enabled, so
 * this clears once the controller successfully sees 11-bit recessive sequences. */
bool can_hw_is_busoff(can_bus_t bus);

/* ---- ESC PWM on TIM1 (1 us/tick, 20 ms period). ---- */
typedef enum {
    ESC_CH_ENGINE = 0,    /* TIM1_CH1 PA8  engine 1100..1900 us */
    ESC_CH_LOAD   = 1,    /* TIM1_CH3 PA10 load   1000..2000 us */
} esc_channel_t;

void esc_hw_init   (esc_channel_t ch, uint16_t idle_us);
void esc_hw_set_us (esc_channel_t ch, uint16_t pulse_us);

/* ---- ECU UART (USART2, 115200 8N1, MegaSquirt). ---- */
typedef enum {
    ECU_UART_OK        = 0,
    ECU_UART_TIMEOUT   = 1,
    ECU_UART_HAL_ERROR = 2,
} ecu_uart_status_t;

ecu_uart_status_t ecu_uart_hw_send(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
ecu_uart_status_t ecu_uart_hw_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
void              ecu_uart_hw_flush_rx(void);

#endif /* PERIPH_WRAPPERS_H */
