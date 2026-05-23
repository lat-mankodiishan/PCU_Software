/*
 * ecu_task.c — Loweheiser ECU on USART2 (PA2/PA3, 115200 8N1).
 *
 * Protocol: MegaSquirt/TunerStudio "A" command — host sends one byte 'A',
 * ECU responds with a fixed-size binary block. The block layout is defined
 * by the [OutputChannels] section of the ECU's TunerStudio .ini; we mirror
 * a curated subset into g_pt.ecu_*.
 *
 * Field decoding is table-driven (s_fields[] below). Adding a new channel
 * is three steps:
 *   1. Add an enum entry to `ecu_field_id_t`.
 *   2. Add a row to `s_fields[]` with offset / type / factor / adder.
 *   3. Add the corresponding g_pt mirror assignment in the parse block.
 *
 * UART HAL access is wrapped in periph_wrappers (ecu_uart_hw_*). Blocking
 * RX is acceptable here — the task runs at osPriorityNormal, and at
 * 115200 baud a 212-byte read costs ~18 ms; the scheduler still preempts
 * via SysTick during HAL_UART_Receive's polling wait.
 */

#include "ecu_task.h"
#include "periph_wrappers.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>

#define ECU_POLL_PERIOD_MS    100        /* 10 Hz polling */
#define ECU_TX_TIMEOUT_MS      20        /* 1 byte ≪ 1 ms @ 115200; 20 ms is generous */
#define ECU_RX_TIMEOUT_MS     250        /* 212 B ≈ 18 ms ideal; allow ECU latency */
#define ECU_STALE_MS         1000        /* declare ECU lost after 1 s of failed polls */

/* ochBlockSize from mainController.ini [OutputChannels]. */
#define ECU_BLOCK_SIZE        212

/* ---- Firmware build mode toggles ------------------------------------ *
 * The MS2/Extra firmware can be built with `#set CELSIUS` (factor 0.05555
 * / adder -320) or default Fahrenheit (factor 0.100 / adder 0). The
 * encoding on the wire is fixed at compile-time of the firmware, not by
 * the runtime tempUnits constant — that only changes TS's display.
 *
 * If the bench readout for ecu_cht_C shows ~80-120 with a cold engine,
 * the firmware is in Fahrenheit mode — flip to 0 and reflash. */
#define ECU_TEMP_CELSIUS_MODE   1

#if ECU_TEMP_CELSIUS_MODE
#  define ECU_TEMP_FACTOR    0.05555f
#  define ECU_TEMP_ADDER    -320.0f
#else
#  define ECU_TEMP_FACTOR    0.100f
#  define ECU_TEMP_ADDER     0.0f
#endif

/* EGT thermocouple amplifier — 0-1250 °C K-type (1.222 °C per ADC count)
 * is the EGTFULL build; 0-1000 °C K-type uses 0.956. */
#define ECU_EGT_FACTOR_C     1.222f      /* alt: 0.956f for 1000 °C range */

/* ---- Field table ---------------------------------------------------- */

typedef enum {
    ECU_FT_U08, ECU_FT_S08,
    ECU_FT_U16, ECU_FT_S16,
    ECU_FT_U32, ECU_FT_S32,
} ecu_ftype_t;

typedef struct {
    const char  *name;        /* matches the INI channel name; for debug */
    uint16_t     offset;      /* byte offset into the response block */
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

/* Order in the table is independent of byte offset — keep this aligned
 * with the parse block below so future readers can match table → mirror
 * by index. */
static const ecu_field_t s_fields[ECU_F_COUNT] = {
    [ECU_F_RPM]          = { "rpm",    6, ECU_FT_U16, 1.000f,            0.0f             },
    [ECU_F_INJ_PW_US]    = { "pw1",    2, ECU_FT_U16, 0.666f,            0.0f             }, /* µs (raw is 0.666 µs/LSB) */
    [ECU_F_CHT_C]        = { "clt",   22, ECU_FT_S16, ECU_TEMP_FACTOR,   ECU_TEMP_ADDER   },
    [ECU_F_IAT_C]        = { "mat",   20, ECU_FT_S16, ECU_TEMP_FACTOR,   ECU_TEMP_ADDER   },
    [ECU_F_EGT_C]        = { "egt6", 128, ECU_FT_U16, ECU_EGT_FACTOR_C,  0.0f             }, /* adc6 × 1.222 = °C */
    [ECU_F_TPS_PCT]      = { "tps",   24, ECU_FT_S16, 0.100f,            0.0f             }, /* % */
    [ECU_F_MAP_KPA]      = { "map",   18, ECU_FT_S16, 0.100f,            0.0f             }, /* kPa */
    [ECU_F_BATT_V]       = { "vbat",  26, ECU_FT_S16, 0.100f,            0.0f             }, /* V */
    [ECU_F_ENGINE_BYTE]  = { "estat", 11, ECU_FT_U08, 1.000f,            0.0f             }, /* raw byte; bit 0 = ready, 1 = crank, 2 = startw, 3 = warmup, 4 = tpsaccaen, 5 = tpsaccden, 6 = mapaccaen, 7 = mapaccden */
    [ECU_F_SYNC_CNT]     = { "sync",  94, ECU_FT_U08, 1.000f,            0.0f             },
};

/* ---- Static state --------------------------------------------------- */
static StaticTask_t s_tcb;
static StackType_t  s_stack[384];        /* 1.5 KB */
static uint8_t      s_block[ECU_BLOCK_SIZE];

/* Diagnostic counters — read via Live Watch. */
volatile uint32_t g_ecu_poll_attempts = 0;
volatile uint32_t g_ecu_poll_ok       = 0;
volatile uint32_t g_ecu_poll_tx_fail  = 0;
volatile uint32_t g_ecu_poll_rx_fail  = 0;

static void ecu_task(void *arg);

/* ---- Big-endian decoder -------------------------------------------- *
 * MS2/Extra wire format is `endianness = big` in the INI. */
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

/* ---- Start ---------------------------------------------------------- */
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

/* ---- Task body ------------------------------------------------------ */
static void ecu_task(void *arg) {
    (void)arg;

    /* Startup grace — let the ECU's serial side settle before first poll. */
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
            /* --- Run all fields through the decoder table --- */
            float eng[ECU_F_COUNT];
            for (uint8_t i = 0; i < ECU_F_COUNT; ++i) {
                eng[i] = decode_field(&s_fields[i], s_block);
            }

            /* --- Fahrenheit-firmware post-conversion --- *
             * In Celsius-mode firmware the table's factor/adder already
             * yield whole °C. In Fahrenheit mode the table yields °F;
             * convert to °C here before storing. EGT comes from adc6 and
             * its multiplier already gives °C directly (regardless of the
             * coolant/IAT compile-time switch). */
#if !ECU_TEMP_CELSIUS_MODE
            eng[ECU_F_CHT_C] = (eng[ECU_F_CHT_C] - 32.0f) * 5.0f / 9.0f;
            eng[ECU_F_IAT_C] = (eng[ECU_F_IAT_C] - 32.0f) * 5.0f / 9.0f;
#endif

            /* --- Mirror into g_pt under one mutex acquisition --- */
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
            /* ecu_fuel_rate_dg_s left at 0 — TODO once injector flow rate is known */
            g_pt.ecu_input_tick    = osKernelGetTickCount();
            osMutexRelease(g_pt_mtx);

            pt_clear_fault(FAULT_ECU_STALE);
            g_ecu_poll_ok++;
        }

        /* Independent staleness check — a single failed poll shouldn't
         * raise the fault; only sustained absence of fresh data should. */
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
