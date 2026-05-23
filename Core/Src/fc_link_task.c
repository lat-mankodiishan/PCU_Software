/* fc_link_task — DroneCAN gateway: Pixhawk <-> PCU, owns libcanard. */

#include "fc_link_task.h"

/* No-op stub until dronecan_dsdlc + libcanard are present. */
#if __has_include("canard.h") && __has_include("dsdl.lat.powertrain.PTConcise.h")

#include "can_manager.h"
#include "periph_wrappers.h"
#include "powertrain_state.h"
#include "sensor_task.h"
#include "canard.h"
#include "dsdl.lat.powertrain.PTConcise.h"
#include "dsdl.lat.powertrain.ThrottleDemand.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.GetNodeInfo_req.h"
#include "uavcan.protocol.GetNodeInfo_res.h"
#include "uavcan.equipment.power.BatteryInfo.h"
#include "uavcan.equipment.esc.RawCommand.h"
#include "uavcan.equipment.ice.reciprocating.Status.h"
#include "uavcan.equipment.ice.FuelTankStatus.h"
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
#define FC_LINK_PT_PERIOD_MS      50            /* 20 Hz PTConcise */
#define FC_LINK_BATT_PERIOD_MS   100            /* 10 Hz */
#define FC_LINK_ICE_PERIOD_MS    100            /* 10 Hz -> EFI_STATUS MAVLink */
#define FC_LINK_FUEL_PERIOD_MS  1000            /*  1 Hz */
#define FC_LINK_FC_STALE_MS     1000

#define FC_LINK_RX_QUEUE_DEPTH    64
#define FC_LINK_MEM_POOL_BYTES  8192

#define FC_LINK_SW_VERSION_MAJOR   1
#define FC_LINK_SW_VERSION_MINOR   0
#define FC_LINK_HW_VERSION_MAJOR   1
#define FC_LINK_HW_VERSION_MINOR   0
#define FC_LINK_NODE_NAME          "lat.pcu"

/* STM32F4 96-bit UID; first 12 of 16 unique_id bytes, rest zero. */
#define STM32_UID_BASE             0x1FFF7A10UL

/* RawCommand int14 indexed by ESC channel; AP CAN_D1_UC_ESC_BM bit N -> chan N+1. */
#define FC_LINK_ESC_INDEX          0u
#define FC_LINK_ESC_RAW_MAX        8191    /* int14 forward full-scale */

static StaticTask_t       s_tcb;
static StackType_t        s_stack[512];          /* 2 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[FC_LINK_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static uint8_t            s_mem_pool[FC_LINK_MEM_POOL_BYTES];
static CanardInstance     s_canard;

static uint8_t            s_tid_pt     = 0;
static uint8_t            s_tid_status = 0;
static uint8_t            s_tid_batt   = 0;
static uint8_t            s_tid_ice    = 0;
static uint8_t            s_tid_fuel   = 0;

static volatile uint32_t  s_fc_last_status_tick = 0;
static uint32_t           s_boot_tick           = 0;

/* DEBUG: per-DTID receive counters. */
volatile uint32_t g_fc_link_rx_nodestatus  = 0;
volatile uint32_t g_fc_link_rx_throttle    = 0;
volatile uint32_t g_fc_link_rx_rawcommand  = 0;
volatile uint32_t g_fc_link_rx_total       = 0;

/* GCS-settable flight_state, index 0, via uavcan.protocol.param.GetSet. */
volatile int8_t g_pcu_param_flight_state = (int8_t)VESC_MODE_IDLE;

/* DEBUG: GetSet service path counters. */
volatile uint32_t g_pcu_should_accept_param = 0;
volatile uint32_t g_pcu_param_handler_calls = 0;
volatile uint32_t g_pcu_param_set_count    = 0;
volatile uint32_t g_pcu_param_resp_rc      = 0;

/* DEBUG: per-publication counters. */
volatile uint32_t g_pub_ice_calls          = 0;
volatile  int32_t g_pub_ice_last_rc        = 0;
volatile uint32_t g_pub_ice_encoded_bytes  = 0;
volatile uint32_t g_pub_fuel_calls         = 0;
volatile  int32_t g_pub_fuel_last_rc       = 0;
volatile uint32_t g_pub_batt_calls         = 0;
volatile  int32_t g_pub_batt_last_rc       = 0;

/* ---- libcanard callbacks ---- */

static bool should_accept(const CanardInstance *ins,
                          uint64_t              *out_signature,
                          uint16_t               data_type_id,
                          CanardTransferType     transfer_type,
                          uint8_t                source_node_id) {
    (void)ins; (void)source_node_id;

    if (transfer_type == CanardTransferTypeBroadcast) {
        if (data_type_id == DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_ID) {
            *out_signature = DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_SIGNATURE;
            return true;
        }
        if (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID) {
            *out_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
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

/* Index 0: flight_state int8 [0..VESC_MODE_FAULT]. */
#define FC_LINK_PARAM_FLIGHT_STATE_NAME      "flight_state"
#define FC_LINK_PARAM_FLIGHT_STATE_NAME_LEN  (sizeof(FC_LINK_PARAM_FLIGHT_STATE_NAME) - 1)

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
    bool by_name = (greq.name.len == FC_LINK_PARAM_FLIGHT_STATE_NAME_LEN) &&
                   (memcmp(greq.name.data,
                           FC_LINK_PARAM_FLIGHT_STATE_NAME,
                           FC_LINK_PARAM_FLIGHT_STATE_NAME_LEN) == 0);
    bool by_index = (greq.name.len == 0) && (greq.index == 0);

    if (by_name || by_index) {
        /* SET when value tag != EMPTY; we accept INTEGER only. */
        if (greq.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
            int64_t v = greq.value.integer_value;
            if (v < 0)                    v = 0;
            if (v > (int64_t)VESC_MODE_FAULT) v = (int64_t)VESC_MODE_FAULT;
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

    if (transfer->data_type_id == DSDL_LAT_POWERTRAIN_THROTTLEDEMAND_ID) {
        g_fc_link_rx_throttle++;
        struct dsdl_lat_powertrain_ThrottleDemand msg;
        /* decoder returns false on success. */
        if (!dsdl_lat_powertrain_ThrottleDemand_decode(transfer, &msg)) {
            /* flight_phase enum == vesc_mode_t. */
            vesc_mode_t mode  = (vesc_mode_t)msg.flight_phase;
            uint16_t    thr   = (uint16_t)(msg.throttle_pct * 100.0f);
            if (thr > 10000) thr = 10000;
            pt_set_fc_inputs(mode, thr);
        }
    }
    else if (transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID) {
        g_fc_link_rx_rawcommand++;
        struct uavcan_equipment_esc_RawCommand msg;
        if (!uavcan_equipment_esc_RawCommand_decode(transfer, &msg)) {
            if (msg.cmd.len > FC_LINK_ESC_INDEX) {
                int16_t raw = msg.cmd.data[FC_LINK_ESC_INDEX];
                /* Clamp to forward range; no reverse for generator. */
                if (raw < 0)                    raw = 0;
                if (raw > FC_LINK_ESC_RAW_MAX)  raw = FC_LINK_ESC_RAW_MAX;
                /* int14 0..8191 -> 0..10000 (0.01 %/LSB). */
                uint16_t thr = (uint16_t)(((uint32_t)raw * 10000u)
                                          / FC_LINK_ESC_RAW_MAX);
                /* Flight state from GCS param, not throttle channel. */
                vesc_mode_t mode = (vesc_mode_t)g_pcu_param_flight_state;
                pt_set_fc_inputs(mode, thr);
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

static void publish_pt_concise(void) {
    powertrain_state_t pt;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    pt = g_pt;
    osMutexRelease(g_pt_mtx);

    struct dsdl_lat_powertrain_PTConcise msg;
    msg.egu_ok           = !(pt.fault_bits & (FAULT_RECT_OFFLINE | FAULT_BUS2_BUSOFF));
    msg.batt_ok          = (pt.bms_soc_pct > 1000);                   /* TODO placeholder */
    msg.v_bus            = pt.rect_state.V_dc_cV   / 100.0f;
    msg.i_load           = 0.0f;                                      /* TODO ADS1262 */
    msg.i_bat            = 0.0f;                                      /* TODO BMS */
    msg.i_rect           = pt.rect_state.I_dc_cA   / 100.0f;
    msg.batt_soc         = pt.bms_soc_pct          / 100.0f;
    msg.fuel_consumption = 0.0f;                                      /* TODO ECU */

    uint8_t buf[DSDL_LAT_POWERTRAIN_PTCONCISE_MAX_SIZE];
    uint32_t len = dsdl_lat_powertrain_PTConcise_encode(&msg, buf);

    canardBroadcast(&s_canard,
                    DSDL_LAT_POWERTRAIN_PTCONCISE_SIGNATURE,
                    DSDL_LAT_POWERTRAIN_PTCONCISE_ID,
                    &s_tid_pt,
                    CANARD_TRANSFER_PRIORITY_MEDIUM,
                    buf, (uint16_t)len);
}

/* uavcan.equipment.power.BatteryInfo: stock MP/QGC battery widget. */
static void publish_battery_info(void) {
    powertrain_state_t pt;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    pt = g_pt;
    osMutexRelease(g_pt_mtx);

    struct uavcan_equipment_power_BatteryInfo msg;
    memset(&msg, 0, sizeof(msg));

    msg.voltage             = pt.rect_state.V_dc_cV / 100.0f;
    msg.current             = 0.0f;                     /* TODO BMS */
    msg.temperature         = 0.0f;
    msg.state_of_charge_pct = (uint8_t)(pt.bms_soc_pct / 100u);
    msg.state_of_health_pct = 100;
    msg.battery_id          = 0;
    msg.model_instance_id   = 0;
    msg.status_flags        = UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATUS_FLAG_IN_USE;

    uint8_t buf[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
    uint32_t len = uavcan_equipment_power_BatteryInfo_encode(&msg, buf);

    g_pub_batt_calls++;
    g_pub_batt_last_rc = canardBroadcast(&s_canard,
                    UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                    UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                    &s_tid_batt,
                    CANARD_TRANSFER_PRIORITY_MEDIUM,
                    buf, (uint16_t)len);
}

/* uavcan.equipment.ice.reciprocating.Status -> AP -> MAVLink EFI_STATUS. */
static void publish_ice_status(void) {
    powertrain_state_t pt;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    pt = g_pt;
    osMutexRelease(g_pt_mtx);

    struct uavcan_equipment_ice_reciprocating_Status msg;
    memset(&msg, 0, sizeof(msg));

    /* DEBUG/MOCK ECU feed; replace with pt.ecu_* once ECU task is online. */
    uint16_t load = pt.fc_throttle_dem_pct / 100u;
    if (load > 100u) load = 100u;

    uint32_t up_s     = (osKernelGetTickCount() - s_boot_tick) / 1000u;
    uint32_t mock_rpm = 1000u + ((uint32_t)load * 50u) + (up_s % 60u) * 10u;
    float    mock_cht_C = 60.0f + ((float)load * 0.5f) + (float)(up_s % 30u);

    /* Always RUNNING; AP EFI driver suppresses updates if STOPPED. */
    msg.state = UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_STATE_RUNNING;
    msg.flags = UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_TEMPERATURE_SUPPORTED;

    msg.engine_load_percent = (uint8_t)load;
    msg.engine_speed_rpm    = mock_rpm;
    msg.coolant_temperature = mock_cht_C + 273.15f; /* K */

    /* Unmeasured -> NaN. */
    msg.intake_manifold_temperature  = NAN;
    msg.atmospheric_pressure_kpa     = NAN;
    msg.intake_manifold_pressure_kpa = NAN;
    msg.spark_dwell_time_ms          = NAN;

    uint8_t  buf[UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_MAX_SIZE];
    uint32_t len = uavcan_equipment_ice_reciprocating_Status_encode(&msg, buf);

    g_pub_ice_calls++;
    g_pub_ice_encoded_bytes = len;
    g_pub_ice_last_rc = canardBroadcast(&s_canard,
                    UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_ID,
                    &s_tid_ice,
                    CANARD_TRANSFER_PRIORITY_MEDIUM,
                    buf, (uint16_t)len);
}

/* uavcan.equipment.ice.FuelTankStatus -> AP fuel telemetry. */
static void publish_fuel_tank_status(void) {
    powertrain_state_t pt;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    pt = g_pt;
    osMutexRelease(g_pt_mtx);

    struct uavcan_equipment_ice_FuelTankStatus msg;
    memset(&msg, 0, sizeof(msg));

    /* DEBUG/MOCK: replace with ECU feed when online. */
    uint16_t load = pt.fc_throttle_dem_pct / 100u;
    if (load > 100u) load = 100u;
    uint32_t up_min = ((osKernelGetTickCount() - s_boot_tick) / 1000u) / 60u;
    uint32_t pct    = (up_min < 100u) ? (100u - up_min) : 0u;
    msg.available_fuel_volume_percent = (uint8_t)pct;
    msg.available_fuel_volume_cm3     = (float)pct * 100.0f;
    msg.fuel_consumption_rate_cm3pm   = 50.0f + (float)load * 4.5f;

    msg.fuel_temperature = NAN;
    msg.fuel_tank_id     = 0;

    uint8_t  buf[UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_MAX_SIZE];
    uint32_t len = uavcan_equipment_ice_FuelTankStatus_encode(&msg, buf);

    g_pub_fuel_calls++;
    g_pub_fuel_last_rc = canardBroadcast(&s_canard,
                    UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_ID,
                    &s_tid_fuel,
                    CANARD_TRANSFER_PRIORITY_MEDIUM,
                    buf, (uint16_t)len);
}

static void publish_node_status(uint32_t uptime_sec) {
    struct uavcan_protocol_NodeStatus msg;
    msg.uptime_sec                  = uptime_sec;
    msg.health                      = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode                        = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    msg.sub_mode                    = 0;
    /* DEBUG: surface RawCommand RX count; restore to pt_get_faults() later. */
    msg.vendor_specific_status_code = (uint16_t)g_fc_link_rx_rawcommand;

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
    uint32_t last_pt_tx     = s_boot_tick;
    uint32_t last_batt_tx   = s_boot_tick;
    uint32_t last_ice_tx    = s_boot_tick;
    uint32_t last_fuel_tx   = s_boot_tick;
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

        /* DEBUG: high-rate pubs gated; re-enable after GetSet verified. */
        (void)last_pt_tx; (void)last_batt_tx; (void)last_ice_tx; (void)last_fuel_tx;
        if (0 && now - last_pt_tx >= FC_LINK_PT_PERIOD_MS) {
            publish_pt_concise();
            last_pt_tx = now;
        }
        if (0 && now - last_batt_tx >= FC_LINK_BATT_PERIOD_MS) {
            publish_battery_info();
            last_batt_tx = now;
        }
        if (0 && now - last_ice_tx >= FC_LINK_ICE_PERIOD_MS) {
            publish_ice_status();
            last_ice_tx = now;
        }
        if (0 && now - last_fuel_tx >= FC_LINK_FUEL_PERIOD_MS) {
            publish_fuel_tank_status();
            last_fuel_tx = now;
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
