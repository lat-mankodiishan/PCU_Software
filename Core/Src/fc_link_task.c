/* fc_link_task — DroneCAN gateway: Pixhawk <-> PCU, owns libcanard. */

#include "fc_link_task.h"

/* No-op stub until dronecan_dsdlc + libcanard are present. */
#if __has_include("canard.h") && __has_include("uavcan.protocol.NodeStatus.h")

#include "can_manager.h"
#include "periph_wrappers.h"
#include "powertrain_state.h"
#include "sensor_task.h"
#include "canard.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo_req.h"
#include "uavcan.protocol.GetNodeInfo_res.h"
#include "uavcan.equipment.esc.RawCommand.h"
#include "uavcan.equipment.actuator.ArrayCommand.h"
#include "uavcan.equipment.actuator.Command.h"
#include "uavcan.protocol.param.GetSet_req.h"
#include "uavcan.protocol.param.GetSet_res.h"
#include "uavcan.protocol.param.Value.h"
#include "uavcan.protocol.param.NumericValue.h"
#include "uavcan.protocol.param.Empty.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>

#define FC_LINK_PERIOD_MS         50            /* 20 Hz */
#define FC_LINK_HEARTBEAT_MS    1000            /* 1 Hz NodeStatus */
#define FC_LINK_FLEX_PERIOD_MS   200            /*  5 Hz FlexDebug batch */
#define FC_LINK_FC_STALE_MS     1000

/* dronecan.protocol.FlexDebug — DTID 16371, single-frame float32 per id.
 * AP needs CAN_D1_UC_OPTION |= 512 (ENABLE_FLEX_DEBUG) to cache for Lua. */
#define FLEXDEBUG_ID                16371u
#define FLEXDEBUG_SIGNATURE         0xECA60382FF038F39ULL
#define FLEXDEBUG_MAX_FRAME_PAYLOAD 7u           /* 8 - 1 tail byte */

/* Per-field FlexDebug message IDs — must match pcu_telem.lua on the FC. */
#define FLEX_ID_DUT  1u
#define FLEX_ID_THR  2u
#define FLEX_ID_IRS  3u
#define FLEX_ID_IRM  4u
#define FLEX_ID_VDC  5u
#define FLEX_ID_IBF  6u
#define FLEX_ID_IBR  7u
#define FLEX_ID_RPM  8u
#define FLEX_ID_IGT  9u
#define FLEX_ID_PRC 10u
#define FLEX_ID_EST 11u
#define FLEX_ID_FLT 12u

/* Set to 1 to publish synthetic test values (each field distinct, step every
 * 3 s) instead of g_pt snapshots. Strip when peripherals come online. */
#define FC_LINK_FLEX_TEST_MODE       1
#define FC_LINK_FLEX_TEST_PERIOD_MS  3000u

#define FC_LINK_RX_QUEUE_DEPTH    64
#define FC_LINK_MEM_POOL_BYTES  8192

#define FC_LINK_SW_VERSION_MAJOR   1
#define FC_LINK_SW_VERSION_MINOR   0
#define FC_LINK_HW_VERSION_MAJOR   1
#define FC_LINK_HW_VERSION_MINOR   0
#define FC_LINK_NODE_NAME          "lat.pcu"

/* STM32F4 96-bit UID; first 12 of 16 unique_id bytes, rest zero. */
#define STM32_UID_BASE             0x1FFF7A10UL

/* RawCommand int14 indexed by ESC channel; AP CAN_D1_UC_ESC_BM bit N -> chan N+1.
 * Copter: average all channels present. Per-motor commands carry attitude trim
 * which cancels out across the rotor array, leaving collective throttle. */
#define FC_LINK_ESC_RAW_MAX        8191    /* int14 forward full-scale */

/* ArrayCommand actuator_id 8 = engine_state from Lua RC bridge (SERVO8 via
 * CAN_D1_UC_SRV_BM). Not motor-arming-gated. AP sends UNITLESS [-1,+1] by
 * default, or PWM (us) if CAN_D1_UC_OPTION USE_ACTUATOR_PWM bit is set. */
#define FC_LINK_ESTATE_ACTUATOR_ID         8u
#define FC_LINK_ESTATE_UNITLESS_CRANK_LO  (-0.5f)
#define FC_LINK_ESTATE_UNITLESS_RUN_LO     (0.5f)
#define FC_LINK_ESTATE_PWM_CRANK_LO        1300.0f
#define FC_LINK_ESTATE_PWM_RUN_LO          1700.0f

static StaticTask_t       s_tcb;
static StackType_t        s_stack[512];          /* 2 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[FC_LINK_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static uint8_t            s_mem_pool[FC_LINK_MEM_POOL_BYTES];
static CanardInstance     s_canard;

static uint8_t            s_tid_status = 0;
static uint8_t            s_tid_flex   = 0;

static volatile uint32_t  s_fc_last_status_tick = 0;
static uint32_t           s_boot_tick           = 0;

/* DEBUG: per-DTID receive counters. */
volatile uint32_t g_fc_link_rx_nodestatus    = 0;
volatile uint32_t g_fc_link_rx_rawcommand    = 0;
volatile uint32_t g_fc_link_rx_arraycommand  = 0;
volatile uint32_t g_fc_link_rx_total         = 0;

/* DEBUG: last engine-state command value seen on actuator_id 8 (for bench tuning
 * the band thresholds). type=0 unitless, type=4 PWM us. */
volatile uint8_t  g_fc_link_estate_cmd_type  = 0;
volatile float    g_fc_link_estate_cmd_value = 0.0f;

/* GCS-settable flight_state, index 0, via uavcan.protocol.param.GetSet. */
volatile int8_t g_pcu_param_flight_state = (int8_t)MODE_IDLE;

/* DEBUG: GetSet service path counters. */
volatile uint32_t g_pcu_should_accept_param = 0;
volatile uint32_t g_pcu_param_handler_calls = 0;
volatile uint32_t g_pcu_param_set_count    = 0;
volatile uint32_t g_pcu_param_resp_rc      = 0;

/* ---- libcanard callbacks ---- */

static bool should_accept(const CanardInstance *ins,
                          uint64_t              *out_signature,
                          uint16_t               data_type_id,
                          CanardTransferType     transfer_type,
                          uint8_t                source_node_id) {
    (void)ins; (void)source_node_id;

    if (transfer_type == CanardTransferTypeBroadcast) {
        if (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID) {
            *out_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
            return true;
        }
        if (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
            *out_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
            return true;
        }
        if (data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
            *out_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
            return true;
        }
        return false;
    }

    if (transfer_type == CanardTransferTypeRequest) {
        if (data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID) {
            *out_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        if (data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_ID) {
            g_pcu_should_accept_param++;
            *out_signature = UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_SIGNATURE;
            return true;
        }
    }
    return false;
}

static void handle_get_node_info(const CanardRxTransfer *req) {
    struct uavcan_protocol_GetNodeInfoResponse rsp;
    memset(&rsp, 0, sizeof(rsp));

    rsp.status.uptime_sec                  = (osKernelGetTickCount() - s_boot_tick) / 1000u;
    rsp.status.health                      = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    rsp.status.mode                        = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    rsp.status.sub_mode                    = 0;
    rsp.status.vendor_specific_status_code = pt_get_faults();

    rsp.software_version.major                = FC_LINK_SW_VERSION_MAJOR;
    rsp.software_version.minor                = FC_LINK_SW_VERSION_MINOR;
    rsp.software_version.optional_field_flags = 0;
    rsp.software_version.vcs_commit           = 0;
    rsp.software_version.image_crc            = 0;

    rsp.hardware_version.major = FC_LINK_HW_VERSION_MAJOR;
    rsp.hardware_version.minor = FC_LINK_HW_VERSION_MINOR;
    memcpy(rsp.hardware_version.unique_id,
           (const void *)STM32_UID_BASE, 12);
    rsp.hardware_version.certificate_of_authenticity.len = 0;

    rsp.name.len = (uint8_t)(sizeof(FC_LINK_NODE_NAME) - 1);
    memcpy(rsp.name.data, FC_LINK_NODE_NAME, rsp.name.len);

    uint8_t buf[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    uint16_t len = (uint16_t)uavcan_protocol_GetNodeInfoResponse_encode(&rsp, buf);

    /* Service response reuses request's transfer ID. */
    uint8_t tid = req->transfer_id;
    canardRequestOrRespond(&s_canard,
                           req->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID,
                           &tid,
                           CANARD_TRANSFER_PRIORITY_MEDIUM,
                           CanardResponse,
                           buf, len);
}

/* Index 0: flight_state int8 [0..MODE_FAULT]. */
#define FC_LINK_PARAM_FLIGHT_STATE_NAME      "flight_state"
#define FC_LINK_PARAM_FLIGHT_STATE_NAME_LEN  (sizeof(FC_LINK_PARAM_FLIGHT_STATE_NAME) - 1)

/* Index 1: engine_state int8 [0..ENGINE_FAULT]; written via pt_set_engine_state(). */
#define FC_LINK_PARAM_ENGINE_STATE_NAME      "engine_state"
#define FC_LINK_PARAM_ENGINE_STATE_NAME_LEN  (sizeof(FC_LINK_PARAM_ENGINE_STATE_NAME) - 1)

static void handle_param_get_set(const CanardRxTransfer *req) {
    g_pcu_param_handler_calls++;
    struct uavcan_protocol_param_GetSetRequest greq;
    memset(&greq, 0, sizeof(greq));
    if (uavcan_protocol_param_GetSetRequest_decode(req, &greq)) {
        return;
    }

    struct uavcan_protocol_param_GetSetResponse rsp;
    memset(&rsp, 0, sizeof(rsp));

    /* Prefer name match; fall back to index when name empty. */
    bool match_flight = ((greq.name.len == FC_LINK_PARAM_FLIGHT_STATE_NAME_LEN) &&
                         (memcmp(greq.name.data,
                                 FC_LINK_PARAM_FLIGHT_STATE_NAME,
                                 FC_LINK_PARAM_FLIGHT_STATE_NAME_LEN) == 0))
                     || ((greq.name.len == 0) && (greq.index == 0));

    bool match_engine = ((greq.name.len == FC_LINK_PARAM_ENGINE_STATE_NAME_LEN) &&
                         (memcmp(greq.name.data,
                                 FC_LINK_PARAM_ENGINE_STATE_NAME,
                                 FC_LINK_PARAM_ENGINE_STATE_NAME_LEN) == 0))
                     || ((greq.name.len == 0) && (greq.index == 1));

    if (match_flight) {
        /* SET when value tag != EMPTY; we accept INTEGER only. */
        if (greq.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
            int64_t v = greq.value.integer_value;
            if (v < 0)                    v = 0;
            if (v > (int64_t)MODE_FAULT) v = (int64_t)MODE_FAULT;
            g_pcu_param_flight_state = (int8_t)v;
            g_pcu_param_set_count++;
        }

        /* Response: current value + canonical name; default/min/max EMPTY. */
        rsp.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        rsp.value.integer_value     = g_pcu_param_flight_state;
        rsp.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        rsp.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        rsp.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        rsp.name.len = FC_LINK_PARAM_FLIGHT_STATE_NAME_LEN;
        memcpy(rsp.name.data,
               FC_LINK_PARAM_FLIGHT_STATE_NAME,
               FC_LINK_PARAM_FLIGHT_STATE_NAME_LEN);
    } else if (match_engine) {
        if (greq.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
            int64_t v = greq.value.integer_value;
            if (v < 0)                       v = 0;
            if (v > (int64_t)ENGINE_FAULT)  v = (int64_t)ENGINE_FAULT;
            /* Use request path so supervisor fires the swap+prime sequence on
             * CRANK->RUN regardless of source (MP, Lua RC, Live Watch). */
            pt_request_engine_state((engine_state_t)v);
            g_pcu_param_set_count++;
        }

        engine_state_t cur;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        cur = g_pt.engine_state;
        osMutexRelease(g_pt_mtx);

        rsp.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        rsp.value.integer_value     = (int64_t)cur;
        rsp.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        rsp.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        rsp.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        rsp.name.len = FC_LINK_PARAM_ENGINE_STATE_NAME_LEN;
        memcpy(rsp.name.data,
               FC_LINK_PARAM_ENGINE_STATE_NAME,
               FC_LINK_PARAM_ENGINE_STATE_NAME_LEN);
    } else {
        /* Empty response signals end-of-list. */
        rsp.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        rsp.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        rsp.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        rsp.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        rsp.name.len                = 0;
    }

    uint8_t  buf[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t len = (uint16_t)uavcan_protocol_param_GetSetResponse_encode(&rsp, buf);

    uint8_t tid = req->transfer_id;
    /* HIGH priority to beat AP RawCommand before GCS timeout. */
    g_pcu_param_resp_rc = (uint32_t)canardRequestOrRespond(&s_canard,
                           req->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_ID,
                           &tid,
                           CANARD_TRANSFER_PRIORITY_HIGH,
                           CanardResponse,
                           buf, len);
}

static void on_transfer_received(CanardInstance    *ins,
                                 CanardRxTransfer  *transfer) {
    (void)ins;

    g_fc_link_rx_total++;

    if (transfer->transfer_type == CanardTransferTypeRequest) {
        if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_ID) {
            handle_get_node_info(transfer);
            return;
        }
        if (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_ID) {
            handle_param_get_set(transfer);
            return;
        }
    }

    if (transfer->transfer_type != CanardTransferTypeBroadcast) return;

    if (transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID) {
        g_fc_link_rx_rawcommand++;
        struct uavcan_equipment_esc_RawCommand msg;
        if (!uavcan_equipment_esc_RawCommand_decode(transfer, &msg) &&
            msg.cmd.len > 0u) {
            /* Sum forward-clamped channels, divide by count -> collective. */
            uint32_t sum = 0u;
            for (uint8_t i = 0; i < msg.cmd.len; i++) {
                int16_t raw = msg.cmd.data[i];
                if (raw < 0)                    raw = 0;
                if (raw > FC_LINK_ESC_RAW_MAX)  raw = FC_LINK_ESC_RAW_MAX;
                sum += (uint32_t)raw;
            }
            uint32_t avg_raw = sum / msg.cmd.len;
            /* int14 0..8191 -> 0..10000 (0.01 %/LSB). */
            uint16_t thr = (uint16_t)((avg_raw * 10000u) / FC_LINK_ESC_RAW_MAX);
            /* Flight state from GCS param, not throttle channel. */
            flight_mode_t mode = (flight_mode_t)g_pcu_param_flight_state;
            pt_set_fc_inputs(mode, thr);
        }
    }
    else if (transfer->data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID) {
        g_fc_link_rx_arraycommand++;
        struct uavcan_equipment_actuator_ArrayCommand msg;
        if (!uavcan_equipment_actuator_ArrayCommand_decode(transfer, &msg)) {
            for (uint8_t i = 0; i < msg.commands.len; i++) {
                if (msg.commands.data[i].actuator_id != FC_LINK_ESTATE_ACTUATOR_ID) {
                    continue;
                }
                const uint8_t ctype = msg.commands.data[i].command_type;
                const float   cval  = msg.commands.data[i].command_value;
                g_fc_link_estate_cmd_type  = ctype;
                g_fc_link_estate_cmd_value = cval;

                engine_state_t req;
                if (ctype == UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM) {
                    if      (cval < FC_LINK_ESTATE_PWM_CRANK_LO) req = ENGINE_OFF;
                    else if (cval < FC_LINK_ESTATE_PWM_RUN_LO)   req = ENGINE_CRANK;
                    else                                          req = ENGINE_RUN;
                } else {
                    if      (cval < FC_LINK_ESTATE_UNITLESS_CRANK_LO) req = ENGINE_OFF;
                    else if (cval < FC_LINK_ESTATE_UNITLESS_RUN_LO)   req = ENGINE_CRANK;
                    else                                               req = ENGINE_RUN;
                }
                pt_request_engine_state(req);
                break;
            }
        }
    }
    else if (transfer->data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
        g_fc_link_rx_nodestatus++;
        s_fc_last_status_tick = osKernelGetTickCount();
        pt_clear_fault(FAULT_FC_STALE);
    }
}

/* ---- TX side ---- */

/* FlexDebug payload: uint16 id LE + uint8[<=5] (TAO, single-frame). */
typedef struct {
    uint16_t    flex_id;
    float       value;
} flex_field_t;

static void publish_flex_debug(uint16_t flex_id, const uint8_t *u8, uint8_t u8_len) {
    if (u8_len > FLEXDEBUG_MAX_FRAME_PAYLOAD - 2u) {
        u8_len = (uint8_t)(FLEXDEBUG_MAX_FRAME_PAYLOAD - 2u);
    }
    uint8_t buf[FLEXDEBUG_MAX_FRAME_PAYLOAD];
    buf[0] = (uint8_t)(flex_id & 0xFFu);
    buf[1] = (uint8_t)((flex_id >> 8) & 0xFFu);
    memcpy(&buf[2], u8, u8_len);

    canardBroadcast(&s_canard,
                    FLEXDEBUG_SIGNATURE,
                    FLEXDEBUG_ID,
                    &s_tid_flex,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buf, (uint16_t)(2u + u8_len));
}

static void publish_flex_debug_float(uint16_t flex_id, float value) {
    uint8_t bytes[4];
    memcpy(bytes, &value, 4);   /* host LE -> wire LE */
    publish_flex_debug(flex_id, bytes, 4u);
}

static void publish_flex_debug_batch(void) {
    powertrain_state_t pt;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    pt = g_pt;
    osMutexRelease(g_pt_mtx);

    const flex_field_t batch[] = {
        { FLEX_ID_DUT, pt.ctl_duty_x10000          / 100.0f },
        { FLEX_ID_THR, pt.engine_throttle_pct_x100 / 100.0f },
        { FLEX_ID_IRS, pt.I_rect_cmd_cA            / 100.0f },
        { FLEX_ID_IRM, pt.rect_state.I_dc_cA       / 100.0f },
        { FLEX_ID_VDC, pt.rect_state.V_dc_cV       / 100.0f },
        { FLEX_ID_IBF, pt.ctl_i_bat_filt_cA        / 100.0f },
        { FLEX_ID_IBR, pt.ctl_i_bat_ref_eff_cA     / 100.0f },
        { FLEX_ID_RPM, (float)pt.rect_state.gen_rpm        },
        { FLEX_ID_IGT, (float)pt.rect_state.igbt_temp_C    },
        { FLEX_ID_PRC, (float)pt.ctl_p_rect_W              },
        { FLEX_ID_EST, (float)pt.engine_state              },
        { FLEX_ID_FLT, (float)pt.fault_bits                },
    };
    for (size_t i = 0; i < sizeof(batch)/sizeof(batch[0]); i++) {
        publish_flex_debug_float(batch[i].flex_id, batch[i].value);
    }
}

#if FC_LINK_FLEX_TEST_MODE
/* Synthetic downlink for bench-testing telemetry plumbing without peripherals.
 * Each field has a distinct base + per-step delta; counter advances every 3 s. */
static void publish_flex_debug_test(void) {
    static uint32_t last_step_tick = 0;
    static uint32_t step           = 0;
    uint32_t now = osKernelGetTickCount();
    if (last_step_tick == 0u || (now - last_step_tick) >= FC_LINK_FLEX_TEST_PERIOD_MS) {
        step++;
        last_step_tick = now;
    }
    const float s = (float)step;
    const flex_field_t batch[] = {
        { FLEX_ID_DUT, 11.0f  + s         },
        { FLEX_ID_THR, 22.0f  + s         },
        { FLEX_ID_IRS, 33.0f  + s         },
        { FLEX_ID_IRM, 44.0f  + s         },
        { FLEX_ID_VDC, 55.0f  + s         },
        { FLEX_ID_IBF, 66.0f  + s         },
        { FLEX_ID_IBR, 77.0f  + s         },
        { FLEX_ID_RPM, 1000.0f + 100.0f*s },
        { FLEX_ID_IGT, 25.0f  + s         },
        { FLEX_ID_PRC, 250.0f + 10.0f*s   },
        { FLEX_ID_EST, (float)(step % 4u) },
        { FLEX_ID_FLT, (float)(step & 0xFFu) },
    };
    for (size_t i = 0; i < sizeof(batch)/sizeof(batch[0]); i++) {
        publish_flex_debug_float(batch[i].flex_id, batch[i].value);
    }
}
#endif

static void publish_node_status(uint32_t uptime_sec) {
    struct uavcan_protocol_NodeStatus msg;
    msg.uptime_sec                  = uptime_sec;
    msg.health                      = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode                        = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    msg.sub_mode                    = 0;
    msg.vendor_specific_status_code = pt_get_faults();

    uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    uint32_t len = uavcan_protocol_NodeStatus_encode(&msg, buf);

    canardBroadcast(&s_canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &s_tid_status,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buf, (uint16_t)len);
}

static void drain_canard_tx(void) {
    const CanardCANFrame *frame;
    while ((frame = canardPeekTxQueue(&s_canard)) != NULL) {
        can_frame_t f;
        f.id  = frame->id & CANARD_CAN_EXT_ID_MASK;
        f.ext = (frame->id & CANARD_CAN_FRAME_EFF) ? 1 : 0;
        f.rtr = (frame->id & CANARD_CAN_FRAME_RTR) ? 1 : 0;
        f.dlc = frame->data_len;
        memcpy(f.data, frame->data, frame->data_len);
        if (can_mgr_send(CAN_BUS_DRONECAN, &f, 0)) {
            canardPopTxQueue(&s_canard);
        } else {
            break;
        }
    }
}

/* ---- Task ---- */

static void fc_link_task(void *arg) {
    (void)arg;
    uint32_t last_flex_tx   = s_boot_tick;
    uint32_t last_status_tx = s_boot_tick;
    uint32_t next           = s_boot_tick;

    for (;;) {
        uint32_t now = osKernelGetTickCount();

        /* RX: feed every frame to libcanard (us timestamp). */
        can_frame_t f;
        while (osMessageQueueGet(s_rx_q, &f, NULL, 0) == osOK) {
            CanardCANFrame cf;
            cf.id       = f.id
                        | (f.ext ? CANARD_CAN_FRAME_EFF : 0)
                        | (f.rtr ? CANARD_CAN_FRAME_RTR : 0);
            cf.data_len = f.dlc;
            cf.iface_id = 0;
            memcpy(cf.data, f.data, f.dlc);
            (void)canardHandleRxFrame(&s_canard, &cf, (uint64_t)now * 1000ULL);
        }

        if (now - last_flex_tx >= FC_LINK_FLEX_PERIOD_MS) {
#if FC_LINK_FLEX_TEST_MODE
            publish_flex_debug_test();
#else
            publish_flex_debug_batch();
#endif
            last_flex_tx = now;
        }
        if (now - last_status_tx >= FC_LINK_HEARTBEAT_MS) {
            publish_node_status((now - s_boot_tick) / 1000u);
            last_status_tx = now;
        }

        if (s_fc_last_status_tick &&
            (now - s_fc_last_status_tick) > FC_LINK_FC_STALE_MS) {
            pt_set_fault(FAULT_FC_STALE);
        }

        drain_canard_tx();
        canardCleanupStaleTransfers(&s_canard, (uint64_t)now * 1000ULL);
        can_hw_diag_snapshot();

        next += FC_LINK_PERIOD_MS;
        osDelayUntil(next);
    }
}

void fc_link_task_start(void) {
    s_boot_tick = osKernelGetTickCount();

    static const osMessageQueueAttr_t qattr = {
        .name    = "fc_rxq",
        .cb_mem  = &s_rxq_cb,
        .cb_size = sizeof(s_rxq_cb),
        .mq_mem  = s_rxq_items,
        .mq_size = sizeof(s_rxq_items),
    };
    s_rx_q = osMessageQueueNew(FC_LINK_RX_QUEUE_DEPTH,
                               sizeof(can_frame_t), &qattr);

    /* Match-all ext frames; libcanard filters by DTID. */
    can_mgr_subscribe(CAN_BUS_DRONECAN, 0u, 0u, true, s_rx_q);

    canardInit(&s_canard, s_mem_pool, sizeof(s_mem_pool),
               on_transfer_received, should_accept, NULL);
    canardSetLocalNodeID(&s_canard, FC_LINK_NODE_ID);

    static const osThreadAttr_t tattr = {
        .name       = "fc_link",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityAboveNormal,           /* prio 4 */
    };
    osThreadNew(fc_link_task, NULL, &tattr);
}

#else

void fc_link_task_start(void) { /* stub until codecs generated */ }

#endif
