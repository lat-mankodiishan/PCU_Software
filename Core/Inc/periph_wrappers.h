#ifndef PERIPH_WRAPPERS_H
#define PERIPH_WRAPPERS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CAN_BUS_DRONECAN = 0,   /* CAN1 — PCU<->FC */
    CAN_BUS_ENGINE   = 1,   /* CAN2 — Rectifier+ECU+Laptop */
    CAN_BUS_COUNT
} can_bus_t;

typedef struct {
    uint32_t id;            /* 11- or 29-bit */
    uint8_t  ext : 1;
    uint8_t  rtr : 1;
    uint8_t  dlc : 4;       /* 0..8 */
    uint8_t  data[8];
} can_frame_t;

/* Init must be called AFTER HAL CAN init (MX_CANx_Init). */
void can_hw_init(can_bus_t bus);

/* Try to post a frame to a free TX mailbox. Returns true if accepted. Non-blocking. */
bool can_hw_try_post(can_bus_t bus, const can_frame_t *f);

/* Number of free TX mailboxes (0..3). */
uint8_t can_hw_tx_free(can_bus_t bus);

/* Configure a 32-bit ID-mask filter into the next free filter bank.
 * Routes matches to FIFO0. Returns filter bank index, or -1 on no slots. */
int can_hw_add_filter(can_bus_t bus, uint32_t id, uint32_t mask, bool ext);

/* Read one frame from FIFO0. Returns true if a frame was popped. */
bool can_hw_read_fifo0(can_bus_t bus, can_frame_t *out);

/* PDB V1 contactor outputs. true = closed. Both pins must be configured
 * as push-pull GPIO outputs in the .ioc. */
void pdb_hw_write(bool battery_closed, bool rectifier_closed);

/* Refresh the IWDG. Must be called by supervisor_task each tick.
 * Wrapped to keep HAL_* out of the task layer. */
void watchdog_refresh(void);

/* Toggle the status LED. Called from the idle hook for a 1 Hz heartbeat. */
void led_hw_toggle(void);

/* DEBUG: snapshot CAN1's ESR + counters into globals. Cheap (one register
 * read). Watch g_can1_tec / g_can1_boff / g_can1_epvf in the debugger. */
void can_hw_diag_snapshot(void);

/* ---- ESC / servo PWM on TIM1 ----------------------------------------
 * Both channels share TIM1's prescaler/period (1 µs/tick, 20 ms period
 * → 50 Hz). CCR value writes the pulse width in microseconds directly.
 * Set CCR once and TIM1 hardware regenerates the pulse each frame; no
 * refresh task is needed. */
typedef enum {
    ESC_CH_ENGINE = 0,    /* TIM1_CH1 / PA8  — engine throttle servo (1100..1900 µs) */
    ESC_CH_LOAD   = 1,    /* TIM1_CH3 / PA10 — load ESC, Hobbywing 200A (1000..2000 µs) */
} esc_channel_t;

/* Park CCR at idle_us, then start the channel. Idempotent. */
void esc_hw_init   (esc_channel_t ch, uint16_t idle_us);

/* Write a new pulse width to the channel. Caller owns clamping. */
void esc_hw_set_us (esc_channel_t ch, uint16_t pulse_us);

#endif /* PERIPH_WRAPPERS_H */
