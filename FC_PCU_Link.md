# FC ↔ PCU Communication Link

DroneCAN (UAVCAN v0) link between the Cube Orange+ flight controller and the
PCU board. All implemented with **stock ArduPilot firmware on the Pixhawk
side** — no autopilot firmware modifications required.

Status: **functional baseline as of 2026-05-25**. Throttle pipeline,
GCS-settable `flight_state`, and GCS-settable `engine_state` verified
end-to-end. Custom DSDLs (PTConcise / ThrottleDemand) removed in favour
of stock messages.

---

## 1. Goal

The Pixhawk flight controller drives the hybrid powertrain via a DroneCAN
bus shared between PCU, FC, and (optionally) a USB-CAN sniffer. Two
operationally distinct command channels were required:

| Channel | Direction | Purpose | Rate |
|---|---|---|---|
| Throttle demand | FC → PCU | Tracks pilot/autopilot stick / mission throttle output | 50–260 Hz when armed |
| Flight state | GCS → PCU | Operator selects engine / contactor mode (idle, cruise, …) | sporadic, on operator change |

Constraints: **no Pixhawk firmware modifications** (Pixhawk runs unmodified
ArduCopter / ArduPlane / ArduPilot); use only standard DroneCAN messages and
services where possible; custom DSDLs allowed only on the PCU's outbound
side, never as a downlink dependency.

---

## 2. Network topology

```
                   1 Mbps DroneCAN bus, twisted pair, 120 Ω terminators
                                    on Pixhawk + (optionally) at PCU end
   ┌──────────────────────────┐                ┌──────────────────────────┐
   │  Cube Orange+  (node 10) │ ◀─────────────▶│  PCU STM32F405 (node 50) │
   │  ArduPilot 4.x stock     │   CAN_H/CAN_L  │  fc_link_task.c          │
   │  CAN1, 1 Mbps            │   common GND   │  CAN1, 1 Mbps            │
   └──────────────────────────┘                └──────────────────────────┘
            ▲                                                ▲
            │ USB MAVLink + SLCAN                            │ ST-LINK (SWD)
            │ (COM10 / COM11)                                │ for debug
            ▼                                                ▼
        Mission Planner / DroneCAN GUI Tool          STM32CubeIDE
                                                     Live Expressions
```

**Wiring:**
- PCU CAN1 uses `PB8` (CAN1_RX) / `PB9` (CAN1_TX), AF9.
- Pixhawk CAN1 4-pin JST-GH: pin 2 = CAN_H, pin 3 = CAN_L, pin 4 = GND.
- Pixhawk has built-in 120 Ω termination on CAN1.
- PCU end terminated externally if the cable is non-trivially long.
- Common GND between boards is mandatory — diff-pair alone is not enough.

**Bitrate / sample point on PCU (`Core/Src/can.c`, verified 2026-05-27):**
- `Prescaler = 2`, `BS1 = 8 TQ`, `BS2 = 1 TQ`, `SyncJumpWidth = 1 TQ`
- APB1 = 20 MHz → 1.0 Mbps, sample point 90 %.
- `TransmitFifoPriority = ENABLE` — required for correct multi-frame TX
  ordering (libcanard pushes frames sharing one CAN ID; with DISABLE the
  hardware can reorder them and the receiver sees toggle-bit mismatches).
- `AutoRetransmission = ENABLE`, `AutoBusOff = ENABLE`.
- Clock source is HSI (±1 % at 25 °C), which is on the edge of the CAN
  timing budget. Crystal HSE would be preferable for production.

**Node IDs:**
- `10` — Cube Orange+ autopilot (set via `CAN_D1_UC_NODE = 10`).
- `50` — PCU (`#define FC_LINK_NODE_ID 50` in `Core/Inc/fc_link_task.h`).
- `127` — DroneCAN GUI Tool / Mission Planner virtual GCS node (must be
  set explicitly — anonymous mode cannot send service requests).

---

## 3. Channels in use

### 3.1 FC → PCU: Throttle (`uavcan.equipment.esc.RawCommand`)

- **DTID** 1030, **Priority** HIGH (8), 29-bit CAN ID format
  `[priority<<24] | [DTID<<8] | src_node`. From autopilot src=10:
  CAN ID = `0x0804060A`.
- ArduPilot publishes RawCommand only when motors are armed *and* throttle
  output is non-zero. Bench-test workarounds:
  - `DISARM_DELAY = 0` so it doesn't auto-disarm at idle throttle.
  - `ARMING_CHECK = 0` to bypass GPS / EKF / compass pre-arm gates.
  - `BRD_SAFETYENABLE = 0` to bypass safety-switch motor inhibit.
  - Motor Test feature (MP → SETUP → Optional Hardware → Motor Test) sends
    `MAV_CMD_DO_MOTOR_TEST`, drives a motor at a specified throttle for a
    specified duration, no arm needed; works for copter frames.
- `ArduPilot publishes channel 1 throttle when `CAN_D1_UC_ESC_BM = 1`
  (bitmask: bit N enables ESC channel N+1).
- Payload is `int14[<=20] cmd`, indexed by ESC channel. PCU reads
  `cmd[FC_LINK_ESC_INDEX]` (default index 0 = channel 1).
- Reception path in `fc_link_task.c::on_transfer_received`:
  1. Decode the `int14` → clamp to `[0, FC_LINK_ESC_RAW_MAX = 8191]`.
  2. Scale to 0..10000 (0.01 % LSB).
  3. Combine with the GCS-set `flight_state` (next channel).
  4. Call `pt_set_fc_inputs(mode, throttle_pct)` which under `g_pt_mtx`
     writes `g_pt.fc_throttle_dem_pct` and `g_pt.fc_flight_state`.

### 3.2 GCS / FC → PCU: Param service (`uavcan.protocol.param.GetSet`)

PCU exposes a configurable parameter table via the standard DroneCAN
parameter service. Mission Planner and the DroneCAN GUI Tool can fetch
and set parameters by right-clicking node 50 → "Configure parameters".
ArduPilot Lua scripts can also write these via `DroneCAN_Handle` (used
for RC → engine_state binding — see §3.4).

| index | name           | type | range | default |
|---|---|---|---|---|
| 0 | `flight_state` | int  | 0..5  | 0       |
| 1 | `engine_state` | int  | 0..5  | 0       |

`flight_state` integer values map 1:1 to `flight_mode_t`:

| value | mode |
|---|---|
| 0 | `MODE_IDLE` |
| 1 | `MODE_TAKEOFF` |
| 2 | `MODE_CLIMB` |
| 3 | `MODE_CRUISE` |
| 4 | `MODE_LAND` |
| 5 | `MODE_FAULT` |

`engine_state` integer values map 1:1 to `engine_state_t`:

| value | mode |
|---|---|
| 0 | `ENGINE_OFF` |
| 1 | `ENGINE_CRANK` |
| 2 | `ENGINE_WARMUP` |
| 3 | `ENGINE_RUN` |
| 4 | `ENGINE_COOLDOWN` |
| 5 | `ENGINE_FAULT` |

Implementation:
- `should_accept` admits service request `data_type_id == 11` with the
  GetSet signature.
- `handle_param_get_set` (in `fc_link_task.c`) decodes the request,
  matches by name (`"flight_state"` / `"engine_state"`) or by index
  (`0` / `1`), applies any SET, and emits a `GetSetResponse` carrying
  the post-set value + canonical name. Default / min / max are returned
  as EMPTY tags to keep the multi-frame transfer short — this matters
  because the GCS-side timeout can fire before a long response (8+
  frames) clears the bus when autopilot is also publishing RawCommand
  at high rate.
- Response priority is HIGH so it wins arbitration against autopilot's
  RawCommand.
- A SET on `flight_state` lands in `volatile int8_t g_pcu_param_flight_state`;
  the next RawCommand reception reads it and feeds it into
  `pt_set_fc_inputs(mode, throttle)`.
- A SET on `engine_state` is applied immediately via `pt_set_engine_state()`,
  which takes `g_pt_mtx` and updates `g_pt.engine_state` +
  `g_pt.engine_state_tick`. The supervisor task's 10 ms loop reads
  `g_pt.engine_state` and dispatches CRANK / WARMUP / RUN / COOLDOWN /
  OFF behaviour. Note: the GPIO PA7 BLEED→LOCK→RUN swap sequence in
  `supervisor_task` is *only* triggered by the bench switch's rising
  edge; direct CRANK→RUN via param SET skips BLEED. This is fine for
  bench bring-up but will need reconciliation before airborne RC use.

On copter, `flight_state` is still settable but downstream code does
not consume it (single controller regardless of phase). Flight-phase
SOC management strategies land on fixed-wing.

### 3.3 PCU → Pixhawk / GCS

| Frame | DTID | Period | Status |
|---|---|---|---|
| `uavcan.protocol.NodeStatus` | 341 | 1 Hz | ✅ enabled |
| `uavcan.protocol.GetNodeInfo` (response) | 1 | on request | ✅ enabled |
| `uavcan.protocol.param.GetSet` (response) | 11 | on request | ✅ enabled |
| `uavcan.protocol.debug.KeyValue` | 16370 | 5 Hz × 12 keys | ✅ enabled (sole telemetry channel) |

### Telemetry architecture decision (2026-05-26)

All powertrain telemetry to the GCS goes through `uavcan.protocol.debug.KeyValue`.
The stock-message publishers (`BatteryInfo`, `ice.reciprocating.Status`,
`FuelTankStatus`, custom `PTConcise`) have been **deleted entirely** rather
than kept gated.

Rationale:
- Battery V/I/SOC for failsafes comes from the **Hobbywing ESC telem on CAN2**,
  forwarded by ArduPilot directly to MAVLink `ESC_TELEMETRY_*` and onward to MP.
  No PCU-side battery publish is needed.
- ICE Status / FuelTankStatus would have required real ECU + fuel-sensor data
  to be useful; we don't have either yet, and §6.5 multi-frame CRC issues bit
  whenever they ran at any meaningful rate.
- KeyValue is self-describing (key string + float value), single-frame
  per broadcast when keys are ≤3 chars, and ArduPilot forwards each as
  MAVLink `NAMED_VALUE_FLOAT` → renderable in MP Quick tab and logged in
  the FC dataflash as `NVF` entries.

### KeyValue field list (PCU → GCS)

Each tick (200 ms) publishes the full set; all keys are 3 chars to keep
each broadcast single-frame.

| Key | Source field in `g_pt` | Unit |
|---|---|---|
| `DUT` | `ctl_duty_x10000` | % |
| `THR` | `engine_throttle_pct_x100` | % |
| `IRS` | `I_rect_cmd_cA` | A (setpoint) |
| `IRM` | `rect_state.I_dc_cA` | A (measured) |
| `VDC` | `rect_state.V_dc_cV` | V |
| `IBF` | `ctl_i_bat_filt_cA` | A (filtered) |
| `IBR` | `ctl_i_bat_ref_eff_cA` | A (reference) |
| `RPM` | `rect_state.gen_rpm` | — |
| `IGT` | `rect_state.igbt_temp_C` | °C |
| `PRC` | `ctl_p_rect_W` | W |
| `EST` | `engine_state` | enum 0..5 |
| `FLT` | `fault_bits` | bitfield as int |

To add a field: edit the `batch[]` array in `publish_debug_kv_batch()`
in `fc_link_task.c`. Keep the new key ≤3 chars. Update this table.

### Deleted publishers (do not resurrect)

- `dsdl.lat.powertrain.PTConcise` (custom DTID 20100) — replaced by KeyValue. Deleted 2026-05-25.
- `dsdl.lat.powertrain.ThrottleDemand` (custom DTID 20101) — replaced by stock `esc.RawCommand`. Deleted 2026-05-25.
- `uavcan.equipment.power.BatteryInfo` (DTID 1092) — battery data sourced from ESC telem on CAN2. Deleted 2026-05-26.
- `uavcan.equipment.ice.reciprocating.Status` (DTID 1120) — engine RPM / temps go via KeyValue. Deleted 2026-05-26.
- `uavcan.equipment.ice.FuelTankStatus` (DTID 1129) — no fuel sensor yet; when added, publish as KeyValue. Deleted 2026-05-26.

---

## 4. PCU-side implementation

All DroneCAN logic lives in `Core/Src/fc_link_task.c`. The task owns the
single `CanardInstance` — no other task touches libcanard. RX frames come
in via the `can_manager` dispatch task on `CAN_BUS_DRONECAN` (CAN1); TX
frames are drained from libcanard's queue into `can_mgr_send`.

Key state:

| Variable | Purpose |
|---|---|
| `s_canard` | The singleton CanardInstance |
| `s_mem_pool[8192]` | libcanard's internal block pool — bumped from 2 KB to 8 KB to handle multi-frame service responses |
| `s_rx_q` | 64-deep RX queue (bumped from 32) |
| `g_pcu_param_flight_state` | Live value of the `flight_state` parameter |

Key callbacks:

| Callback | Behavior |
|---|---|
| `should_accept` | Filters by data type ID. Currently admits NodeStatus broadcasts, RawCommand broadcasts, ThrottleDemand broadcasts (legacy custom DSDL), GetNodeInfo requests, and GetSet requests. |
| `on_transfer_received` | Dispatches by transfer type and DTID. Service requests go to `handle_get_node_info` or `handle_param_get_set`; broadcasts go to per-DTID branches. |

Periodic publish loop runs at `FC_LINK_PERIOD_MS = 50` ms (20 Hz). On each
tick, drains libcanard's TX queue into the can_manager, then sleeps via
`osDelayUntil`.

Outside of fc_link_task:

| File | Role |
|---|---|
| `Core/Src/can_manager.c` | Per-bus TX/RX queues, ISR drain, subscription registry. `TX_QUEUE_DEPTH = 64`. |
| `Core/Src/periph_wrappers.c` | Thin HAL_CAN wrapper used by can_manager. |
| `Core/Src/can.c` | CubeMX-generated CAN1/CAN2 init (1 Mbps timing) — do not regenerate without preserving manual edits. |
| `Core/Src/can_callbacks.c` | HAL_CAN ISR callbacks routing into can_manager. |
| `Core/Src/powertrain_state.c` | Single `g_pt` shared state struct + helpers (`pt_set_fc_inputs`, `pt_get_faults`, …). All access guarded by `g_pt_mtx`. |
| `Core/Src/supervisor_task.c` | Reads `g_pt.fc_throttle_dem_pct` + `g_pt.fc_flight_state`, runs the control law, writes `g_pt.I_rect_cmd_cA`. |
| `Core/Src/rectifier_task.c` | Reads `g_pt.I_rect_cmd_cA`, encodes a VESC current-demand frame, sends on `CAN_BUS_ENGINE` (CAN2). |

### Throttle path, end-to-end

```
ArduPilot RawCommand (CAN1, src=10)
  └─ fc_link_task::on_transfer_received
        decode int14 → clamp → scale → mode = g_pcu_param_flight_state
        └─ pt_set_fc_inputs(mode, thr)
              └─ g_pt.fc_throttle_dem_pct, g_pt.fc_flight_state  [under g_pt_mtx]

supervisor_task (5 ms loop)
  └─ snapshots g_pt.fc_throttle_dem_pct → control_law_step()
  └─ pt_set_setpoint(I_rect_cmd_cA, mode)
        └─ g_pt.I_rect_cmd_cA

rectifier_task (CAN2 / CAN_BUS_ENGINE)
  └─ snapshots g_pt.I_rect_cmd_cA
  └─ vesc_proto_encode_curr_dem(...) → CAN frame on CAN2 to VESC rectifier
```

### GCS parameter path, end-to-end

```
DroneCAN GUI Tool (node 127) / Mission Planner
  └─ uavcan.protocol.param.GetSet request (DTID 11), dest=50
       single-frame request with index=0 + new value

PCU CAN1 receive
  └─ libcanard reassembles transfer
  └─ fc_link_task::handle_param_get_set
        match by name "flight_state" or index 0
        if SET (value tag = INTEGER): clamp 0..MODE_FAULT,
                                       g_pcu_param_flight_state = v
        build response (value + name only, default/min/max EMPTY)
        canardRequestOrRespond(... CanardResponse, HIGH priority)

  → next time RawCommand arrives, mode field uses new g_pcu_param_flight_state
```

---

## 5. Pixhawk-side configuration

Set in Mission Planner → CONFIG → Full Parameter List, Write Params,
power-cycle Pixhawk. **No firmware build required** — these are stock
ArduPilot params.

| Parameter | Value | Effect |
|---|---|---|
| `CAN_P1_DRIVER` | `1` | Bind CAN1 hardware to driver slot 1 |
| `CAN_P1_BITRATE` | `1000000` | 1 Mbps to match PCU |
| `CAN_D1_PROTOCOL` | `1` | DroneCAN on driver 1 |
| `CAN_D1_UC_NODE` | `10` | Autopilot's own DroneCAN node ID |
| `CAN_D1_UC_ESC_BM` | `1` | Publish channel 1 throttle as RawCommand |
| `BRD_SAFETYENABLE` | `0` | Disable hardware safety inhibit (bench only) |
| `ARMING_CHECK` | `0` | Bypass pre-arm gates (bench only) |
| `DISARM_DELAY` | `0` | Disable auto-disarm at idle throttle (bench only) |

⚠️ The last three are **bench-only**. Restore to ArduPilot defaults
(`BRD_SAFETYENABLE=1`, `ARMING_CHECK=1`, `DISARM_DELAY=10`) before flight.

---

## 6. Issues encountered and how they were resolved

### 6.1 Mission Planner's DroneCAN node inspector is unreliable
**Symptom:** MP's "DroneCAN/UAVCAN" inspector shows only its own node 127;
PCU's NodeStatus never appears even though the autopilot's CAN1 is
receiving it.
**Resolution:** Use the standalone **DroneCAN GUI Tool** (`pip install
dronecan_gui_tool`) connected to the Pixhawk's USB SLCAN port (`COM11`).
Or use the slcan_sniff.py at `Documents/PCU_CAN_test/slcan_sniff.py`.
MP's tab is purely a UI rendering bug.

### 6.2 ECAN Tools System Time is unreliable at capture start
**Symptom:** Frame rates appear 100× too fast right after opening the bus.
**Resolution:** Trust `uptime_sec` from NodeStatus payloads as ground
truth, not the System Time column. The first ~2000 captured rows share a
single timestamp because of USB-buffer flushing.

### 6.3 STM32CubeProgrammer pollutes our BSS region during reads
**Symptom:** Reading PCU RAM via `STM32_Programmer_CLI -r32` (HOTPLUG or
NORMAL mode) returns Thumb-2-instruction-looking bytes instead of our
counter values; same address gives different bytes between reads.
**Resolution:** STM32CubeProgrammer loads its SWD loader into our BSS area.
Either use `mode=UR` (under reset) for clean DAP-direct reads (but resets
the chip), or use **Live Expressions in CubeIDE** which talks GDB and
doesn't inject any RAM loader. Live Expressions is the canonical path.

### 6.4 CubeIDE doesn't import our CMake project natively
**Symptom:** "Import → C/C++" only shows "Existing Code as Makefile
Project" in this CubeIDE 1.19 patch level.
**Resolution:** A thin wrapper `Makefile` at the project root forwards
`make all` and `make clean` to `cmake --build build/Debug`. Then importing
as a Makefile project works. CubeIDE drives CMake transparently.

### 6.5 Multi-frame TX issues with concurrent publishers
**Symptom:** ICE Status (DTID 1120) arrived at the GCS with CRC mismatches;
GetSet responses timed out before completing.
**Root cause:** Autopilot's RawCommand at ~390 Hz dominates the bus, and
PCU's libcanard memory pool (originally 2 KB) plus shallow can_manager TX
queue (16) couldn't sustain multiple concurrent multi-frame transfers.
**Resolution applied:**
- Bumped `FC_LINK_MEM_POOL_BYTES` from 2048 to 8192.
- Bumped `TX_QUEUE_DEPTH` in can_manager from 16 to 64.
- Made GetSet response carry only `value + name` (default/min/max EMPTY)
  to fit in fewer frames.
- Bumped GetSet response priority from MEDIUM to HIGH.
- All multi-frame publishers eventually deleted (2026-05-25, 2026-05-26)
  in favour of single-frame KeyValue. The multi-frame fragility
  diagnosis remains valid for any future multi-frame transfer added
  back — see §3.3.
**Not yet fixed:** ICE Status full multi-frame round-trip — set aside.
The CRC mismatch on assembly across long transfers under bus pressure
needs further investigation (possibly a TAO setting mismatch or pool
fragmentation).

### 6.6 DroneCAN GUI Tool's anonymous mode silently drops service calls
**Symptom:** "Fetch All" in the parameter editor times out with no PCU-side
counter increment.
**Resolution:** In the GUI Tool's setup dialog, set **Local Node ID = 127**
(any 1..126 not colliding with 10 / 50 also works). Anonymous nodes
(node ID = 0) cannot initiate service calls per the DroneCAN spec.

### 6.7 Pixhawk auto-disarms within 5 seconds during bench testing
**Resolution:** Set `DISARM_DELAY = 0` to disable the idle-throttle
disarm timer. Restore to default before flight.

### 6.8 Motor test denied
**Resolution:** `BRD_SAFETYENABLE = 0` (bench only).

### 6.9 EFI_STATUS not populating in Mission Planner
**Symptom:** Even with `EFI_TYPE = DroneCAN`, MP's MAVLink Inspector shows
EFI_STATUS as a placeholder under "vehicle 2" but no field updates.
**Diagnosis:** ArduPilot's EFI subsystem is not consuming PCU's ICE
Status frames, almost certainly because of the same multi-frame CRC
mismatch in §6.5. Set aside in favor of the simpler param-service route
for flight state.

---

## 7. Verification setup

### Live observation from the PC

1. Plug the Cube Orange+ USB into the laptop. It enumerates as two
   COM ports: `COM10` (MAVLink) and `COM11` (SLCAN bridge).
2. Mission Planner connects to `COM10` for autopilot params and HUD.
3. **Close MP** before opening the SLCAN port for raw frame sniffing
   (they're mutually exclusive).
4. Standalone CAN sniffing options:
   - `python -m dronecan_gui_tool` → SLCAN, `COM11`, 1 Mbps, local node
     **127** → opens the Bus Monitor + node list + parameter editor.
   - `python Documents/PCU_CAN_test/slcan_sniff.py` → 15-second capture,
     summarizes per-DTID frame counts and per-node NodeStatus enrollment.
   - Waveshare USB-CAN-B sniffer + ECAN Tools as a third independent
     witness on the bus when SLCAN passthrough hides PCU frames.

### CubeIDE Live Expressions (canonical)

Add these to the Live Expressions panel during a debug session:

```
g_pcu_param_flight_state                    # 0..5 = flight_mode_t
g_pt.fc_throttle_dem_pct                    # 0..10000 (0.01 %)
g_pt.fc_flight_state                        # current mode
g_pt.I_rect_cmd_cA                          # rectifier-side current setpoint

# DroneCAN telemetry
g_fc_link_rx_rawcommand                     # ticks at autopilot's RawCommand rate
g_fc_link_rx_nodestatus                     # ticks at autopilot's NodeStatus rate
g_pcu_param_handler_calls                   # ticks per GetSet request received
g_pcu_param_set_count                       # ticks per SET (not GET) variant
g_pcu_param_resp_rc                         # frames enqueued by last response (≥1 = good)

# CAN1 health
g_can1_tec                                  # 0 = clean, 96+ = warning, 256 = bus-off
g_can1_rec
g_can1_boff
g_can1_tx_post_ok
g_can1_tx_post_no
```

---

## 8. Cleanup TODO before production / merge

- [x] ~~Delete custom DSDLs `PTConcise` (20100) and `ThrottleDemand`
      (20101) + generated codec files + dead code paths.~~ Done
      2026-05-25.
- [x] ~~Delete stock publishers `BatteryInfo` / `ice.reciprocating.Status`
      / `FuelTankStatus` and consolidate all telemetry through KeyValue.~~
      Done 2026-05-26 — see §3.3 "Telemetry architecture decision".
- [ ] Remove all debug counters in `fc_link_task.c`:
      `g_fc_link_rx_*`, `g_pcu_should_accept_param`,
      `g_pcu_param_handler_calls`, `g_pcu_param_set_count`,
      `g_pcu_param_resp_rc`.
- [ ] Remove `g_can1_*` ESR snapshot variables and `can_hw_diag_snapshot()`
      in `periph_wrappers.c`/`.h` and the call in `fc_link_task` loop.
- [ ] Restore NodeStatus's `vendor_specific_status_code = pt_get_faults()`
      (currently hijacked to surface `g_fc_link_rx_rawcommand` for
      SWD-free observation).
- [ ] Populate the KeyValue values from real `g_pt.bms_*` / `g_pt.ecu_*`
      fields once BMS and ECU tasks are online (today the rect/ctl
      fields are real but bms/ecu paths land back at zero).
- [ ] Restore Pixhawk-side `BRD_SAFETYENABLE = 1`, `ARMING_CHECK = 1`,
      `DISARM_DELAY = 10` (or whatever your flight-config defaults are)
      before any flight.
- [ ] Investigate and fix the ICE Status (DTID 1120) multi-frame CRC
      mismatch if EFI telemetry is required. Likely candidates: TAO flag
      mismatch between encoder and Python receiver, pool fragmentation
      under bus pressure, or libcanard version-specific bit-packing bug.
- [ ] Reconcile the GPIO PA7 BLEED→LOCK→RUN swap sequence with
      param-SET driven `engine_state` transitions. Today the swap only
      fires on the bench switch's rising edge; an airborne RC switch
      writing `engine_state=ENGINE_RUN` via param SET skips BLEED.
      Either run BLEED on *any* CRANK→RUN transition, or make the RC
      script write CRANK first + trigger swap separately.
- [ ] Optional: implement `uavcan.protocol.param.ExecuteOpcode SAVE` to
      persist `flight_state` / `engine_state` across PCU power-cycles
      (writes to one backup FLASH sector or to the RTC backup
      registers). Without this, both params reset to 0 on every boot.

---

## 9. Useful scripts and tools generated during this work

| Path | Purpose |
|---|---|
| `Documents/PCU_CAN_test/slcan_sniff.py` | Headless sniff of the Pixhawk's USB SLCAN port; tallies DTID counts and NodeStatus enrollment. |
| `Documents/PCU_CAN_test/watch_pcu_rawcommand.py` | Watches PCU's NodeStatus and decodes the embedded RawCommand counter from `vendor_specific_status_code`. |
| Project root `Makefile` | Wrapper so STM32CubeIDE's "Existing Code as Makefile Project" import drives our CMake build. |

---

## 10. References

- DroneCAN specification: <https://dronecan.github.io/Specification/>
- libcanard v0 (vendored at `Middlewares/Third_Party/libcanard/`)
- Auto-generated codecs: `Middlewares/Third_Party/dronecan_codec/{include,src}/`
- ArduPilot DroneCAN driver docs: <https://ardupilot.org/copter/docs/common-uavcan-setup-advanced.html>
- DroneCAN GUI Tool: <https://github.com/DroneCAN/gui_tool>
