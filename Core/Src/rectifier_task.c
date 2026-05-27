#include "rectifier_task.h"
#include "can_manager.h"
#include "vesc_proto.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define RECT_PERIOD_MS         5            /* 200 Hz */
#define RECT_RX_QUEUE_DEPTH    8
#define RECT_STALE_MS          200

static StaticTask_t       s_tcb;
static StackType_t        s_stack[256];     /* 1 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[RECT_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static void rectifier_task(void *arg);

static inline int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void rectifier_task_start(void) {
    static const osMessageQueueAttr_t qattr = {
        .name    = "rect_rxq",
        .cb_mem  = &s_rxq_cb,
        .cb_size = sizeof(s_rxq_cb),
        .mq_mem  = s_rxq_items,
        .mq_size = sizeof(s_rxq_items),
    };
    s_rx_q = osMessageQueueNew(RECT_RX_QUEUE_DEPTH,
                               sizeof(can_frame_t),
                               &qattr);

    can_mgr_subscribe(CAN_BUS_ENGINE,
                      VESC_ID_GET_RECT_STATE_CONCISE,
                      0x7FFu, /* exact-match 11-bit */
                      false,
                      s_rx_q);

    static const osThreadAttr_t tattr = {
        .name       = "rectifier",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityAboveNormal,     /* 4 */
    };
    osThreadNew(rectifier_task, NULL, &tattr);
}

/* 0x104/0x105: on-change + 1 s keep-alive; VESC dedups. */
#define MOTOR_TYPE_KEEPALIVE_TICKS  200u   /* 200 x 5 ms */
#define INVERT_DIR_KEEPALIVE_TICKS  200u

static void rectifier_task(void *arg) {
    (void)arg;
    uint8_t   seq  = 0;
    uint8_t   mt_seq = 0;
    uint8_t   id_seq = 0;
    uint32_t  next = osKernelGetTickCount();
    vesc_motor_type_t prev_mt = (vesc_motor_type_t)0xFFu; /* force first TX */
    uint8_t           prev_id = 0xFFu;                    /* force first TX */
    uint32_t mt_ticks_since_tx = MOTOR_TYPE_KEEPALIVE_TICKS;
    uint32_t id_ticks_since_tx = INVERT_DIR_KEEPALIVE_TICKS;
    can_frame_t f;

    for (;;) {
        /* ---- RX drain ---- */
        while (osMessageQueueGet(s_rx_q, &f, NULL, 0) == osOK) {
            vesc_rect_state_t s;
            if (vesc_proto_decode_rect_state_concise(&f, &s) != VESC_DECODE_OK)
                continue;
            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.rect_state      = s;
            g_pt.rect_state_tick = osKernelGetTickCount();
            osMutexRelease(g_pt_mtx);
            pt_clear_fault(FAULT_RECT_STALE);
        }

        /* ---- TX snapshot + clamp; one of 0x101/0x102/0x103 per tick ---- */
        rect_ctrl_mode_t mode_now;
        int16_t   I_cA_now;
        int32_t   omega_now;
        int16_t   duty_now;
        flight_mode_t pt_mode;
        vesc_motor_type_t mt_now;
        uint8_t   id_now;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        mode_now  = g_pt.rect_ctrl_mode;
        I_cA_now  = clamp_i16(g_pt.I_rect_cmd_cA,    -I_RECT_MAX_CA,   I_RECT_MAX_CA);
        omega_now = clamp_i32(g_pt.omega_e_cmd_erpm, -OMEGA_E_MAX_ERPM, OMEGA_E_MAX_ERPM);
        duty_now  = clamp_i16(g_pt.duty_cmd_x10000,  -DUTY_MAX_X10000,  DUTY_MAX_X10000);
        pt_mode   = g_pt.mode;
        mt_now    = g_pt.rect_motor_type;
        id_now    = g_pt.rect_invert_direction ? 1u : 0u;
        osMutexRelease(g_pt_mtx);

        const uint8_t seq_now = seq++;
        can_frame_t tx;
        switch (mode_now) {
        case RECT_CTRL_OMEGA: {
            vesc_omega_dem_t cmd = { .omega_e_cmd_erpm = omega_now,
                                     .mode = pt_mode, .seq = seq_now };
            vesc_proto_encode_omega_dem(&cmd, &tx);
            break;
        }
        case RECT_CTRL_DUTY: {
            vesc_duty_dem_t cmd = { .duty_cmd_x10000 = duty_now,
                                    .mode = pt_mode, .seq = seq_now };
            vesc_proto_encode_duty_dem(&cmd, &tx);
            break;
        }
        case RECT_CTRL_CURRENT:
        default: {
            vesc_curr_dem_t cmd = { .I_rect_cmd_cA = I_cA_now,
                                    .mode = pt_mode, .seq = seq_now };
            vesc_proto_encode_curr_dem(&cmd, &tx);
            break;
        }
        }
        (void)can_mgr_send(CAN_BUS_ENGINE, &tx, 0);

        /* ---- 0x104 motor-type: on-change + keep-alive ---- */
        mt_ticks_since_tx++;
        if (mt_now != prev_mt || mt_ticks_since_tx >= MOTOR_TYPE_KEEPALIVE_TICKS) {
            vesc_motor_type_cmd_t mt_cmd = {
                .motor_type = mt_now,
                .mode       = pt_mode,
                .seq        = mt_seq++,
            };
            can_frame_t mt_tx;
            vesc_proto_encode_motor_type_cmd(&mt_cmd, &mt_tx);
            (void)can_mgr_send(CAN_BUS_ENGINE, &mt_tx, 0);
            prev_mt = mt_now;
            mt_ticks_since_tx = 0;
        }

        /* ---- 0x105 invert-dir: on-change + keep-alive ---- */
        id_ticks_since_tx++;
        if (id_now != prev_id || id_ticks_since_tx >= INVERT_DIR_KEEPALIVE_TICKS) {
            vesc_invert_dir_cmd_t id_cmd = {
                .invert_direction = id_now ? true : false,
                .mode             = pt_mode,
                .seq              = id_seq++,
            };
            can_frame_t id_tx;
            vesc_proto_encode_invert_dir_cmd(&id_cmd, &id_tx);
            (void)can_mgr_send(CAN_BUS_ENGINE, &id_tx, 0);
            prev_id = id_now;
            id_ticks_since_tx = 0;
        }

        /* ---- Staleness check ---- */
        osMutexAcquire(g_pt_mtx, osWaitForever);
        uint32_t last = g_pt.rect_state_tick;
        osMutexRelease(g_pt_mtx);
        if (last && (osKernelGetTickCount() - last) > RECT_STALE_MS) {
            pt_set_fault(FAULT_RECT_STALE);
        }

        next += RECT_PERIOD_MS;
        osDelayUntil(next);
    }
}
