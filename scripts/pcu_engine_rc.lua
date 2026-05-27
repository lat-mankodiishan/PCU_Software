-- pcu_engine_rc.lua — Map a 3-position RC switch to PCU engine_state.
--
-- Path:  RC channel  ->  this script  ->  SERVO8 PWM  ->  AP DroneCAN actuator bridge
--    ->  uavcan.equipment.actuator.ArrayCommand (actuator_id=8)
--    ->  PCU fc_link_task  ->  pt_request_engine_state
--    ->  supervisor processes the request (fires swap+prime on CRANK->RUN).
--
-- ActuatorArrayCommand path is used instead of ESC RawCommand because
-- RawCommand is gated by motor-arming safety, ArrayCommand is not — engine
-- state must be controllable while the copter is disarmed on the bench.
--
-- Required FC params:
--   SCR_ENABLE         = 1
--   SERVO8_FUNCTION    = 94      (Scripting1 — keeps flight code off this channel)
--   SERVO8_MIN         = 1000
--   SERVO8_MAX         = 2000
--   CAN_D1_UC_SRV_BM   = <existing> | 128   (set bit 7 so SERVO8 -> actuator_id 8)
--   CAN_D1_UC_ESC_BM   = <existing>          (DO NOT set bit 7 here)
--
-- PCU side decodes ArrayCommand in fc_link_task.c. AP sends command_type=UNITLESS
-- by default (value in [-1, +1]) or PWM (raw us) if CAN_D1_UC_OPTION bit is set.
-- The handler accepts both and uses these bands:
--   UNITLESS  value < -0.5  -> OFF      (PWM ~1000)
--   UNITLESS  -0.5..+0.5    -> CRANK    (PWM ~1500)
--   UNITLESS  >  +0.5       -> RUN      (PWM ~2000)
--   PWM       value < 1300  -> OFF
--   PWM       1300..1700    -> CRANK
--   PWM       > 1700        -> RUN
--
-- Defaults below assume the engine-state switch is on RC7 (3-pos). Edit RC_CHANNEL
-- and the PWM thresholds if your radio's switch lands elsewhere.

local RC_CHANNEL    = 7
local SERVO_CHAN_0  = 7         -- 0-indexed; SERVO8 = chan 7

local IN_LOW_MAX    = 1300      -- below this = OFF
local IN_MID_MAX    = 1700      -- below this (and >= IN_LOW_MAX) = CRANK; else RUN

local OUT_OFF       = 1000
local OUT_CRANK     = 1500
local OUT_RUN       = 2000

local PERIOD_MS     = 100       -- 10 Hz; PCU samples RawCommand on every TX
local OVERRIDE_MS   = 250       -- safety: if script stops, channel reverts

local last_state    = nil

local function classify(pwm)
    if not pwm or pwm < IN_LOW_MAX then return "OFF",   OUT_OFF
    elseif pwm < IN_MID_MAX        then return "CRANK", OUT_CRANK
    else                                return "RUN",   OUT_RUN
    end
end

local function update()
    local pwm = rc:get_pwm(RC_CHANNEL)
    local state, out = classify(pwm)

    SRV_Channels:set_output_pwm_chan_timeout(SERVO_CHAN_0, out, OVERRIDE_MS)

    if state ~= last_state then
        gcs:send_text(6, string.format("engine_rc: %s (rc=%d -> servo=%d)",
                                       state, pwm or -1, out))
        last_state = state
    end

    return update, PERIOD_MS
end

gcs:send_text(6, "pcu_engine_rc: bridging RC" .. RC_CHANNEL ..
              " -> SERVO" .. (SERVO_CHAN_0 + 1) .. " -> RawCommand[7]")
return update, 1000     -- first tick after 1 s so RC is alive
