/* ecu_task — Loweheiser ECU on USART2 via MegaSquirt 'A' poll, 10 Hz. */

#include "ecu_task.h"
#include "periph_wrappers.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>

#define ECU_POLL_PERIOD_MS    100        /* 10 Hz */
#define ECU_TX_TIMEOUT_MS      20
#define ECU_RX_TIMEOUT_MS     250        /* 212 B ~18 ms + ECU latency */
#define ECU_STALE_MS         1000

/* ochBlockSize from mainController.ini [OutputChannels]. */
#define ECU_BLOCK_SIZE        212

/* 1 = MS2/Extra built with #set CELSIUS; 0 = default Fahrenheit. */
#define ECU_TEMP_CELSIUS_MODE   1

#if ECU_TEMP_CELSIUS_MODE
#  define ECU_TEMP_FACTOR    0.05555f
#  define ECU_TEMP_ADDER    -320.0f
#else
#  define ECU_TEMP_FACTOR    0.100f
#  define ECU_TEMP_ADDER     0.0f
#endif

/* EGTFULL K-type build: 1.222 C/count; 1000 C build uses 0.956. */
#define ECU_EGT_FACTOR_C     1.222f

/* ---- Field table ---- */

typedef enum {
    ECU_FT_U08, ECU_FT_S08,
    ECU_FT_U16, ECU_FT_S16,
    ECU_FT_U32, ECU_FT_S32,
} ecu_ftype_t;

typedef struct {
    const char  *name;        /* INI channel name (debug) */
    uint16_t     offset;      /* byte offset into response */
    ecu_ftype_t  type;
    float        factor;
    float        adder;
} ecu_field_t;

typedef enum {
    ECU_F_RPM = 0,
    ECU_F_INJ_PW_US,
    ECU_F_CHT_C,
    ECU_F_IAT_C,
    ECU_F_EGT_C,
    ECU_F_TPS_PCT,
    ECU_F_MAP_KPA,
    ECU_F_BATT_V,
    ECU_F_ENGINE_BYTE,
    ECU_F_SYNC_CNT,
    ECU_F_COUNT
} ecu_field_id_t;

static const ecu_field_t s_fields[ECU_F_COUNT] = {
    [ECU_F_RPM]          = { "rpm",    6, ECU_FT_U16, 1.000f,            0.0f             },
    [ECU_F_INJ_PW_US]    = { "pw1",    2, ECU_FT_U16, 0.666f,            0.0f             }, /* us, 0.666 us/LSB */
    [ECU_F_CHT_C]        = { "clt",   22, ECU_FT_S16, ECU_TEMP_FACTOR,   ECU_TEMP_ADDER   },
    [ECU_F_IAT_C]        = { "mat",   20, ECU_FT_S16, ECU_TEMP_FACTOR,   ECU_TEMP_ADDER   },
    [ECU_F_EGT_C]        = { "egt6", 128, ECU_FT_U16, ECU_EGT_FACTOR_C,  0.0f             },
    [ECU_F_TPS_PCT]      = { "tps",   24, ECU_FT_S16, 0.100f,            0.0f             }, /* % */
    [ECU_F_MAP_KPA]      = { "map",   18, ECU_FT_S16, 0.100f,            0.0f             }, /* kPa */
    [ECU_F_BATT_V]       = { "vbat",  26, ECU_FT_S16, 0.100f,            0.0f             }, /* V */
    [ECU_F_ENGINE_BYTE]  = { "estat", 11, ECU_FT_U08, 1.000f,            0.0f             }, /* bit0 ready, 1 crank, 2 startw, 3 warmup */
    [ECU_F_SYNC_CNT]     = { "sync",  94, ECU_FT_U08, 1.000f,            0.0f             },
};

static StaticTask_t s_tcb;
static StackType_t  s_stack[384];        /* 1.5 KB */
static uint8_t      s_block[ECU_BLOCK_SIZE];

/* Diagnostic counters. */
volatile uint32_t g_ecu_poll_attempts = 0;
volatile uint32_t g_ecu_poll_ok       = 0;
volatile uint32_t g_ecu_poll_tx_fail  = 0;
volatile uint32_t g_ecu_poll_rx_fail  = 0;

static void ecu_task(void *arg);

/* MS2/Extra wire format is big-endian per INI. */
static int32_t parse_raw(const uint8_t *b, ecu_ftype_t t, uint16_t off) {
    const uint8_t *p = &b[off];
    switch (t) {
    case ECU_FT_U08: return (int32_t)(uint8_t)p[0];
    case ECU_FT_S08: return (int32_t)(int8_t) p[0];
    case ECU_FT_U16: return (int32_t)(uint16_t)((p[0] << 8) | p[1]);
    case ECU_FT_S16: return (int32_t)(int16_t) ((p[0] << 8) | p[1]);
    case ECU_FT_U32: return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
                                    | ((uint32_t)p[2] <<  8) |  (uint32_t)p[3]);
    case ECU_FT_S32: return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
                                    | ((uint32_t)p[2] <<  8) |  (uint32_t)p[3]);
    default:         return 0;
    }
}

static float decode_field(const ecu_field_t *f, const uint8_t *block) {
    int32_t raw = parse_raw(block, f->type, f->offset);
    return (float)raw * f->factor + f->adder;
}

void ecu_task_start(void) {
    static const osThreadAttr_t tattr = {
        .name       = "ecu",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityNormal,         /* prio 3 */
    };
    osThreadNew(ecu_task, NULL, &tattr);
}

static void ecu_task(void *arg) {
    (void)arg;

    /* Startup grace for ECU serial side. */
    osDelay(500);

    uint32_t      next  = osKernelGetTickCount();
    const uint8_t cmd_A = 'A';

    for (;;) {
        g_ecu_poll_attempts++;
        ecu_uart_hw_flush_rx();

        if (ecu_uart_hw_send(&cmd_A, 1, ECU_TX_TIMEOUT_MS) != ECU_UART_OK) {
            g_ecu_poll_tx_fail++;
        } else if (ecu_uart_hw_recv(s_block, ECU_BLOCK_SIZE,
                                    ECU_RX_TIMEOUT_MS) != ECU_UART_OK) {
            g_ecu_poll_rx_fail++;
        } else {
            float eng[ECU_F_COUNT];
            for (uint8_t i = 0; i < ECU_F_COUNT; ++i) {
                eng[i] = decode_field(&s_fields[i], s_block);
            }

#if !ECU_TEMP_CELSIUS_MODE
            eng[ECU_F_CHT_C] = (eng[ECU_F_CHT_C] - 32.0f) * 5.0f / 9.0f;
            eng[ECU_F_IAT_C] = (eng[ECU_F_IAT_C] - 32.0f) * 5.0f / 9.0f;
#endif

            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.ecu_rpm           = (uint16_t)eng[ECU_F_RPM];
            g_pt.ecu_inj_pw_us     = (uint16_t)eng[ECU_F_INJ_PW_US];
            g_pt.ecu_cht_C         = (int16_t) eng[ECU_F_CHT_C];
            g_pt.ecu_iat_C         = (int16_t) eng[ECU_F_IAT_C];
            g_pt.ecu_egt_C         = (int16_t) eng[ECU_F_EGT_C];
            g_pt.ecu_tps_pct_x10   = (int16_t)(eng[ECU_F_TPS_PCT]  * 10.0f);
            g_pt.ecu_map_kPa_x10   = (int16_t)(eng[ECU_F_MAP_KPA]  * 10.0f);
            g_pt.ecu_batt_cV       = (int16_t)(eng[ECU_F_BATT_V]   * 100.0f);
            g_pt.ecu_engine_status = (uint8_t) eng[ECU_F_ENGINE_BYTE];
            g_pt.ecu_sync_err_cnt  = (uint8_t) eng[ECU_F_SYNC_CNT];
            /* TODO: ecu_fuel_rate_dg_s once injector flow rate known. */
            g_pt.ecu_input_tick    = osKernelGetTickCount();
            osMutexRelease(g_pt_mtx);

            pt_clear_fault(FAULT_ECU_STALE);
            g_ecu_poll_ok++;
        }

        /* Staleness check; single failed poll won't raise fault. */
        osMutexAcquire(g_pt_mtx, osWaitForever);
        uint32_t last = g_pt.ecu_input_tick;
        osMutexRelease(g_pt_mtx);
        if (last && (osKernelGetTickCount() - last) > ECU_STALE_MS) {
            pt_set_fault(FAULT_ECU_STALE);
        }

        next += ECU_POLL_PERIOD_MS;
        osDelayUntil(next);
    }
}
