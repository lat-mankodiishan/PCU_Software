-- pcu_telem.lua — Bridge PCU FlexDebug telemetry to MAVLink NAMED_VALUE_FLOAT.
--
-- Stock ArduPilot does not subscribe to uavcan.protocol.debug.KeyValue, but it
-- does cache dronecan.protocol.FlexDebug by (source_node, msg.id). The PCU
-- (node 50) publishes one FlexDebug per telemetry field at 5 Hz, payload =
-- little-endian float32. This script polls each ID on CAN1 and emits a
-- NAMED_VALUE_FLOAT so the value shows up in Mission Planner's Status tab.
--
-- Install: copy to APM/scripts/ on the FC SD card. Requires SCR_ENABLE=1.
-- Field IDs must match FLEX_ID_* in PCU_Code_Ground/Core/Src/fc_link_task.c.

local BUS     = 0       -- CAN1
local NODE    = 50      -- PCU node ID
local PERIOD_MS = 200   -- 5 Hz (match PCU FC_LINK_FLEX_PERIOD_MS)

-- {flex_id, NAMED_VALUE_FLOAT name}
local FIELDS = {
    { 1,  "DUT" },  -- rect duty %
    { 2,  "THR" },  -- engine throttle %
    { 3,  "IRS" },  -- I_rect setpoint A
    { 4,  "IRM" },  -- I_rect measured A
    { 5,  "VDC" },  -- DC bus V
    { 6,  "IBF" },  -- I_bat filtered A
    { 7,  "IBR" },  -- I_bat reference A
    { 8,  "RPM" },  -- generator RPM
    { 9,  "IGT" },  -- IGBT temp C
    { 10, "PRC" },  -- rect power W
    { 11, "EST" },  -- engine state enum
    { 12, "FLT" },  -- fault bitfield
}

local last_us = {}      -- [flex_id] = last timestamp seen
for _, f in ipairs(FIELDS) do last_us[f[1]] = 0 end

local function update()
    for _, f in ipairs(FIELDS) do
        local flex_id, name = f[1], f[2]
        local ts, buf = DroneCAN_get_FlexDebug(BUS, NODE, flex_id, last_us[flex_id])
        if ts and ts ~= last_us[flex_id] and buf and #buf >= 4 then
            last_us[flex_id] = ts
            local value = string.unpack("<f", buf)
            -- MAVLink: MP Tuning view (MAV_<name>) + tlog
            gcs:send_named_float(name, value)
            -- SD BIN log: 4-char msg name "PCU", fields Name (char[16]) + Value (float).
            -- Review post-flight via Mission Planner > DataFlash Logs > message "PCU".
            logger:write("PCU", "Name,Value", "Nf", name, value)
        end
    end
    return update, PERIOD_MS
end

gcs:send_text(6, "pcu_telem: bridging FlexDebug node " .. NODE .. " -> NAMED_VALUE_FLOAT")
return update, 1000     -- first tick after 1 s
