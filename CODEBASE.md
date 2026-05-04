# PCU Firmware — Function-Level Reference

Generated 2026-04-27. Covers every `.c`/`.h` pair under `Core/`, plus
`MIDWARE/`, `dsdl/`, generated codecs, and `Middlewares/Third_Party/libcanard/`.

---

## 0. Layering & Boot Flow

```
Application tasks  ──► Service modules  ──► Hardware drivers  ──► Wrappers  ──► HAL
(supervisor,            (can_manager,       (ads1262,             (periph_     (HAL_*)
 rectifier, pdb,         vesc_proto,         max31855,             wrappers,
 sensor, log,            control_law,        FATFS_SD)             can_callbacks,
 fc_link, bms, ecu)      powertrain_state)                         swo_io)
```

**HAL-isolation discipline:** only `periph_wrappers.c`, `can_callbacks.c`,
the hardware-driver files (`ads1262.c`, `max31855.c`, `FATFS_SD.c`), and
`swo_io.c` may call `HAL_*`. Everything else uses portable APIs.

**Boot order** (`main.c`):
1. `HAL_Init`
2. `SystemClock_Config` — 40 MHz SYSCLK / 20 MHz APB1
3. `MX_*_Init` peripherals: GPIO, CAN1, CAN2, DAC, I2C1, SPI1/2/3, TIM1, FATFS, IWDG (last)
4. `osKernelInitialize`
5. `MX_FREERTOS_Init`:
   - `control_law_self_test()` — pure tests, halts in `Error_Handler` on fail
   - `pt_init()` — mutex + zero `g_pt`
   - `can_mgr_init()` — queues, dispatch task, IRQs armed
   - `rectifier_task_start()`
   - `supervisor_task_start()`
   - `pdb_task_start()`
   - `sensor_task_start()`
   - `log_task_start()`
   - `fc_link_task_start()` (no-op stub if codecs missing)
   - `bms_task_start()`
   - `ecu_task_start()`
   - `defaultTask` created (bench stub)
6. `osKernelStart()` — never returns

---

## 1. CubeMX-Generated Peripheral Init

These files are owned by CubeMX. Don't hand-edit beyond `USER CODE` blocks.

### `main.c` / `main.h`

- **`main()`** — Cortex-M reset entry. Runs HAL init, clock config, all
  `MX_*_Init` peripheral setup, then hands off to FreeRTOS.
- **`SystemClock_Config()`** — PLLM=8, PLLN=80, PLLP=2 → SYSCLK = 40 MHz,
  APB1 = 20 MHz, APB2 = 20 MHz. LSI used for IWDG.
- **`Error_Handler()`** — `__disable_irq()` + infinite loop. Called on init
  failures, asserts, and our hooks.
- **`main.h`** — exports GPIO pin defines: `ADC_RST`, `ADC_START`, `ADC_DRDY`,
  `LED` (PB2), `CS_TC1/2/3`, `CS_ADC`, `CS_SD` (PB6), `PDB_RECT` (PC15),
  `PDB_BAT` (PB11). Plus `Error_Handler` declaration.

### `freertos.c`

User code in `USER CODE` blocks; rest is CubeMX boilerplate.

- **`MX_FREERTOS_Init()`** — runs `control_law_self_test`, then in
  `RTOS_THREADS` block creates all 9 application tasks. Empty
  `defaultTask` is also created here.
- **`StartDefaultTask()`** — bench stub: writes `pt_set_bms_inputs(7000)`
  once, then loops calling `pt_set_fc_inputs(VESC_MODE_CRUISE, 5000)`
  every 1 ms. **Will conflict with `fc_link_task` writes once an FC is
  attached** — disable then.
- **`vApplicationIdleHook()`** — toggles LED on PB2 every 500 ms via
  `led_hw_toggle()`. Freezes if idle never runs (system overloaded).
- **`vApplicationTickHook()`** — decrements `Timer1`/`Timer2` (FATFS_SD
  ms-countdown timeouts). Required because we run `_FS_REENTRANT = 1`.
- **`vApplicationStackOverflowHook()`** — `__disable_irq()` +
  `Error_Handler()`. Inspect `pcTaskName` in debugger to identify the
  guilty task.
- **`vApplicationMallocFailedHook()`** — same shape; fires on
  `pvPortMalloc` failure (libcanard pool exhaustion or task-creation
  starvation).

### `can.c` / `can.h`

- **`MX_CAN1_Init()`** — CAN1 at **1 Mbps** (prescaler=2, BS1=8, BS2=1,
  SJW=1 with 20 MHz APB1). `AutoRetransmission = ENABLE`,
  `AutoBusOff = ENABLE`. PA11/PA12 alt-function.
- **`MX_CAN2_Init()`** — CAN2 at **250 kbps** (prescaler=8). Same flags.
  PB12/PB13 alt-function.
- **`HAL_CAN_MspInit/MspDeInit`** — RCC clock + GPIO + NVIC enables.
- Externs: `hcan1`, `hcan2`.

### `spi.c` / `spi.h`

- **`MX_SPI1_Init()`** — SPI1 to SD card (PB6 CS, PA5/6/7 SCK/MISO/MOSI).
- **`MX_SPI2_Init()`** — SPI2 to MAX31855 thermocouples (PB10 SCK, PB14
  MISO, PB15 MOSI; CS PC6/7/8).
- **`MX_SPI3_Init()`** — SPI3 to ADS1262 (PC10/11/12 SCK/MISO/MOSI;
  CS PD2).

### `i2c.c`, `dac.c`, `tim.c`, `gpio.c`

- **`MX_I2C1_Init`** — I2C1 (currently unused).
- **`MX_DAC_Init`** — DAC (currently unused).
- **`MX_TIM1_Init`** — TIM1 PWM (currently unused; reserved).
- **`MX_GPIO_Init`** — clocks all GPIO ports, sets up `LED`,
  `PDB_BAT`/`PDB_RECT`, `ADC_*`, `CS_*` as outputs (push-pull, low at
  reset, no pull). Also configures `ADC_DRDY` (PC5) as EXTI input.

### `iwdg.c` / `iwdg.h`

- **`MX_IWDG_Init()`** — Prescaler 4 + reload 512 → ~64 ms timeout
  nominal (43.5 ms worst case at fast LSI corner). Started after all
  other peripherals so the rest of boot has the full timeout window
  before the first kick from `supervisor_task`.

### `stm32f4xx_it.c` / `stm32f4xx_it.h`

CubeMX-generated IRQ vector table. Each handler dispatches to the HAL's
own ISR routine (which then calls the appropriate `HAL_*_Callback`
weak function — overridden by us in `can_callbacks.c` and
`sensor_task.c`).

### `stm32f4xx_hal_msp.c`, `stm32f4xx_hal_timebase_tim.c`

- **`HAL_MspInit`** — vendor RCC clock setup, NVIC priority grouping.
- **TIM6 IRQ** — owns `HAL_GetTick()` (FreeRTOS owns SysTick).

### `system_stm32f4xx.c`, `syscalls.c`, `sysmem.c`

Vendor reset vector, default newlib syscall stubs, sbrk-style heap
allocator. Stock CubeMX. `_write` from `syscalls.c` is **weak** so our
`swo_io.c` override wins.

### `FreeRTOSConfig.h`

- `configUSE_PREEMPTION = 1`, `configTICK_RATE_HZ = 1000`,
  `configMAX_PRIORITIES = 7`, `configTOTAL_HEAP_SIZE = 32768` (heap_4,
  reserved for libcanard pool only).
- `configUSE_IDLE_HOOK = 1` — for LED heartbeat.
- `configUSE_TICK_HOOK = 1` — for `Timer1/Timer2` decrement (FATFS_SD).
- `configCHECK_FOR_STACK_OVERFLOW = 2` — full method 2 check.
- `configUSE_MALLOC_FAILED_HOOK = 1`.
- `configSUPPORT_STATIC_ALLOCATION = 1`, `configUSE_NEWLIB_REENTRANT = 1`.

---

## 2. Hardware Wrappers

### `periph_wrappers.c` / `periph_wrappers.h`

Sole owner of HAL CAN, GPIO (for PDB/LED), and IWDG calls outside the
CubeMX-generated init files. Defines portable types
(`can_bus_t`, `can_frame_t`).

- **`handle_of(bus)`** *(static)* — translates `can_bus_t` enum
  (`CAN_BUS_AVIONICS`, `CAN_BUS_POWERTRAIN`) to the HAL's `&hcan1`/`&hcan2`.
  Sole place this mapping lives.
- **`can_hw_init(bus)`** — enables the four IRQs we use (RX FIFO0
  pending, TX mailbox empty, bus-off, error) and calls `HAL_CAN_Start`.
  Filters NOT installed here.
- **`can_hw_try_post(bus, *frame)`** — non-blocking. If a TX mailbox is
  free, hands the frame to `HAL_CAN_AddTxMessage`. Returns `true` on
  accept.
- **`can_hw_tx_free(bus)`** — wraps `HAL_CAN_GetTxMailboxesFreeLevel`
  (0..3).
- **`can_hw_add_filter(bus, id, mask, ext)`** — installs one 32-bit
  ID-mask filter into the next free bank. CAN1 owns banks 0-13, CAN2
  owns 14-27 (`SlaveStartFilterBank=14`). Routes matches to FIFO0.
  Static counter advances per call.
- **`can_hw_read_fifo0(bus, *out)`** — pops one frame from FIFO0 and
  fills `can_frame_t`. Returns `false` on empty FIFO so the caller can
  drain in a loop.
- **`pdb_hw_write(bat_closed, rect_closed)`** — drives the two PDB
  contactor GPIOs (`PB11` battery, `PC15` rectifier). Called only by
  `pdb_task`.
- **`watchdog_refresh()`** — `HAL_IWDG_Refresh(&hiwdg)`. Called by
  `supervisor_task` each 10 ms tick.
- **`led_hw_toggle()`** — toggles `LED_Pin` (PB2). Called by idle hook
  every 500 ms.

### `can_callbacks.c`

HAL → manager bridge. No header.

- **`HAL_CAN_RxFifo0MsgPendingCallback(*h)`** — HAL fires this when
  FIFO0 has a frame. We compare `h` against `&hcan1`/`&hcan2` and
  forward to `can_mgr_isr_rx_fifo0(bus)`.
- **`HAL_CAN_TxMailbox0/1/2CompleteCallback(*h)`** — same pattern,
  forwarding to `can_mgr_isr_tx_complete(bus)`. Mailboxes 1/2 chain to
  mailbox 0's body.

### `swo_io.c`

- **`_write(file, *ptr, len)`** — strong override of newlib's weak
  `_write`. Routes every byte through `ITM_SendChar` on stimulus port 0.
  After this file is linked, plain `printf` shows up in the SWV viewer.
  When the debugger isn't connected, ITM is a fast register-poll no-op.

---

## 3. Hardware Drivers

### `ads1262.c` / `ads1262.h`

TI ADS1262 24-bit delta-sigma ADC over SPI3. Copied verbatim from
PCU_Bringup project. 10-channel scanning with DRDY-driven channel
switching.

- **`ADC_Init(*ch, n, sps, filt)`** — hardware reset → verify chip ID →
  enable internal reference → config interface (CRC checksum) → set
  channel 0 mux → start conversion. Stores user channel config for
  later use.
- **`ADC_Reset()`** — pulse `ADC_RST_Pin` low for 1 ms, wait 10 ms.
- **`ADC_VerifyID()`** — reads ID register, returns `true` if matches
  ADS1262 family.
- **`ADC_SetCS(state)`** / **`ADC_SetStart(state)`** — drives chip
  select / START pins.
- **`ADC_SPIByteTransfer(tx)`** — single-byte SPI transfer wrapper.
- **`ADC_WReg(reg, data)`** / **`ADC_RReg(reg, *data, n)`** — register
  write / read.
- **`ADC_StartADC()` / `ADC_StopADC()`** — sends `START`/`STOP`
  commands.
- **`ADC_SetChannel(idx)`** — programs MODE2 (gain + SPS) and INPMUX
  for the given channel index from the user config.
- **`ADC_DRDY_IRQ()`** — sets `drdy_flag` when DRDY EXTI fires. Called
  by `HAL_GPIO_EXTI_Callback` in `sensor_task.c`.
- **`ADC_DataReady()`** — returns and clears `drdy_flag`.
- **`ADC_ReadRaw(*raw, *status)`** — issues `RDATA`, reads 6-byte
  response (status + 24-bit + checksum), returns checksum-valid flag.
- **`ADC_ConverttoEngUnit(raw, idx)`** — applies stored
  `scale * raw + offset` per channel.
- **`ADC_SwitchToNextChannel()`** — increments `current_ch`, sets
  `scan_complete` when wrapping back to 0, programs the new channel.
- **`ADC_ReadAndStore()`** — reads current channel raw, stores it,
  switches to next.
- **`ADC_GetScanResult(*out)`** — if a scan completed, copies stored
  raw values, applies engineering conversion, returns full result.
- **`ADC_SelfOffset_Cal()` / `ADC_SelfGain_Cal()`** — sends self-cal
  commands; blocks on DRDY.
- **`ADC_DumpRegisters()`** — debug; `printf`s all registers
  (now visible via SWO).

### `max31855.c` / `max31855.h`

3× MAX31855 K-type cold-junction thermocouple ICs on SPI2. Single-shot
read.

- **`MAX_SetCS(ch, state)`** — drives `CS_TC1/2/3` based on channel
  index 0..2.
- **`MAX_ReadSPI(ch, *data)`** — clocks 32 bits in, returns raw word.
- **`MAX_GetData(raw, *reading)`** — pure decoder: extracts 14-bit
  signed thermocouple temp (0.25 °C/LSB), 12-bit signed cold-junction
  temp (0.0625 °C/LSB), and 4 fault flags (open, short-VCC, short-GND,
  any-fault). Returns `true` if no fault.

### `MIDWARE/FATFS_SD/FATFS_SD.c` / `FATFS_SD.h`

SPI-mode SD card driver from Khaled Magdy / DeepBlueMbedded. CS = PB6,
SPI = SPI1.

- **`SD_disk_initialize`/`status`/`read`/`write`/`ioctl`** — implements
  the FatFs `Diskio_drvTypeDef` interface. Called from `user_diskio.c`.
- Internal: `SD_PowerOn`, `SD_SendCmd`, `SD_RxDataBlock`, `SD_TxDataBlock`,
  `SD_ReadyWait`. Uses `Timer1`/`Timer2` ms-countdowns (decremented in
  our `vApplicationTickHook`).

### `FATFS/Target/user_diskio.c`

Thin shim from CubeMX-generated FatFs to our `SD_disk_*` driver.
Forwards `USER_initialize`/`status`/`read`/`write`/`ioctl` calls to
`SD_disk_*`.

---

## 4. Service Modules

### `can_manager.c` / `can_manager.h`

Single owner of TX queues and RX dispatch across both CAN buses.
Static-allocated.

- **`can_mgr_init()`** — for each bus: creates a 16-deep TX queue and a
  32-deep RX queue, zeros the 8-entry subscriber table, calls
  `can_hw_init`. Then creates the static `can_disp` task at
  `osPriorityAboveNormal`.
- **`can_mgr_send(bus, *frame, timeout_ms)`** — fast path: if the TX
  queue is empty *and* a HW mailbox is free (under
  `taskENTER_CRITICAL`), hand the frame straight to hardware.
  Slow path: enqueue; the TX-complete ISR drains.
- **`can_mgr_subscribe(bus, id, mask, ext, dest_q)`** — adds an entry to
  the subscriber table (max 8 per bus) and installs a HW filter for the
  same `(id, mask, ext)`. **Today: one subscribe == one filter bank**;
  no coalescing.
- **`can_mgr_last_rx_tick(bus)`** — returns kernel tick of last RX on
  that bus. Used for per-bus liveness later.
- **`can_mgr_isr_rx_fifo0(bus)`** *(ISR-context)* — drains FIFO0,
  stamps `last_rx_tick`, posts each frame to the bus's `rx_q` for the
  dispatch task.
- **`can_mgr_isr_tx_complete(bus)`** *(ISR-context)* — while a mailbox
  is free and a frame is queued, dequeue and post.
- **`rx_dispatch_task`** *(static)* — at prio 4: drains both buses'
  `rx_q`s, scans the subscriber table, posts to every matching
  subscriber's queue. Polls every 1 ms (replace with thread-flag wakeup
  later).

### `vesc_proto.c` / `vesc_proto.h`

Pure encode/decode for the PCU↔Rectifier link. No HAL, no kernel.
Host-testable. CAN IDs `0x101` (TX) / `0x201` (RX).

- **`vesc_crc8(*buf, len)`** — CRC-8/SMBUS (poly 0x07, init 0x00,
  no reflection). Test vector `"123456789"` → `0xF4`.
- **`vesc_proto_encode_curr_dem(*in, *out_frame)`** — builds the 8-byte
  `SendCurrDem` frame: `int16 I_rect_cmd_cA` (LE), `mode`, `seq`, 3
  reserved, `crc8` over [0..6].
- **`vesc_proto_decode_rect_state_concise(*frame, *out)`** — parses the
  8-byte `GetRectStateConcise` frame: `V_dc_cV`, `I_dc_cA`, `gen_rpm`,
  `igbt_temp_C`, packed `fault_bits[7:4]/seq[3:0]`. Returns
  `VESC_DECODE_OK` / `BAD_ID` / `BAD_LEN`.

### `control_law.c` / `control_law.h`

Pure outer-loop controller. Mode-switched I_rect command. No HAL, no
kernel.

- **`control_law_init(*st)`** — zeros filter and integrator state.
- **`control_law_default_params(*p)`** — fills defaults: peak 120 A,
  full-throttle target 115 A, 5 % deadband, 0.5 A/tick slew, SOC bias
  +20 A under 30 % SOC, throttle EMA α ≈ 6 %/tick (1 Hz cutoff at
  10 ms).
- **`clamp_i32(v, lo, hi)`** *(static inline)*.
- **`control_law_step(*st, *in, *p)`** — one tick of the control law:
  - Update Q15 EMA filter on `throttle_dem_pct`.
  - **TAKEOFF / CLIMB / LAND** → output = `I_rect_peak_cA` (battery
    handles transients).
  - **CRUISE** → if `|throttle_dem - throttle_filt|` ≥ deadband, trim
    integrator by `trim_step_cA` toward
    `I_load_full_cA × throttle_dem / 10000` (+SOC bias if low).
    Otherwise hold.
  - **IDLE / FAULT** → output = 0.
  - Clamp to int16, store, return.

### `control_law_test.c` / `control_law_test.h`

In-firmware self-test. Runs at boot inside `MX_FREERTOS_Init` before
the kernel starts.

- **`control_law_self_test()`** — 10 numbered tests:
  1. TAKEOFF → peak.
  2. CLIMB → peak.
  3. LAND → peak.
  4. IDLE → 0.
  5. FAULT → 0.
  6. CRUISE deadband holds setpoint within ±1 % bump.
  7. CRUISE outside deadband trims by exactly `trim_step_cA`.
  8. CRUISE no SOC bias → target = `I_load_full × dem / 10000`.
  9. CRUISE with SOC bias → target += `soc_bias_cA`.
  10. EMA filter converges on a 1000-tick step.
  On any failure: writes the test ID into
  `control_law_test_failed_id` (volatile, debugger-readable) and calls
  `Error_Handler()`. On pass: clears the ID and returns.

### `powertrain_state.c` / `powertrain_state.h`

The shared state struct + mutex. Exposes typed helpers so callers don't
manually take the mutex.

- **`pt_init()`** — zeros `g_pt`, sets `mode = VESC_MODE_IDLE`, creates
  the static priority-inheriting mutex.
- **`pt_set_setpoint(I_cmd, mode)`** — supervisor's exclusive write
  point for the rectifier setpoint.
- **`pt_set_fault(mask)` / `pt_clear_fault(mask)` / `pt_get_faults()`**
  — atomic bitfield ops on `g_pt.fault_bits`.
- **`pt_set_fc_inputs(flight_state, throttle_dem_pct)`** — writes FC
  side fields and stamps `fc_input_tick`.
- **`pt_set_bms_inputs(soc_pct)`** — bench-stub-friendly partial BMS
  writer (only SOC). Real BMS task should write all four fields
  directly under the mutex.
- **`pt_set_contactor_cmds(bat, rect)`** — supervisor sets desired
  contactor states; pdb_task mirrors to GPIOs.
- `g_pt` definition (`powertrain_state_t`) — fields:
  - **Setpoint**: `I_rect_cmd_cA`, `mode`.
  - **Telemetry**: `rect_state`, `rect_state_tick`.
  - **FC**: `fc_flight_state`, `fc_throttle_dem_pct`, `fc_input_tick`.
  - **BMS**: `bms_soc_pct`, `bms_v_bat_cV`, `bms_i_bat_cA`,
    `bms_max_cell_C`, `bms_input_tick`.
  - **ECU**: `ecu_rpm`, `ecu_fuel_rate_dg_s`, `ecu_cht_C`,
    `ecu_input_tick`.
  - **Liveness**: `supervisor_heartbeat`.
  - **Contactors**: `contactor_battery_cmd`, `contactor_rectifier_cmd`.
  - **Faults**: `fault_bits` (uint16, `fault_id_t` flags).
- `fault_id_t` enum (bits 0..7 allocated): `RECT_STALE`, `RECT_OFFLINE`,
  `FC_STALE`, `BUS1_BUSOFF`, `BUS2_BUSOFF`, `SUPERVISOR_HANG`,
  `BMS_STALE`, `ECU_STALE`. Bits 8..15 free.
- `I_RECT_MAX_CA = 16000` (160 A) — hardware-of-last-resort clamp.

---

## 5. Application Tasks

### `supervisor_task.c` / `supervisor_task.h`

10 ms tick, `osPriorityHigh` (5), 1 KB stack. Sole writer of `g_pt`
setpoint.

- **`supervisor_task_start()`** — initializes static `s_ctl_state` and
  `s_ctl_params` via `control_law_*`, then creates the task.
- **`supervisor_task`** *(static)* loop, every 10 ms:
  1. Snapshot `fc_flight_state`, `fc_throttle_dem_pct`, `bms_soc_pct`,
     `fault_bits` under mutex.
  2. If `RECT_OFFLINE | BUS2_BUSOFF` set → override mode to
     `VESC_MODE_FAULT`.
  3. `control_law_step` → new `I_rect_cmd_cA`.
  4. `pt_set_setpoint(I_cmd, final_mode)`.
  5. `pt_set_contactor_cmds(true, true)` (V1 always-closed; fault-driven
     opens deferred).
  6. Increment `supervisor_heartbeat`.
  7. `watchdog_refresh()` → IWDG kick.
  8. `osDelayUntil` next.

### `rectifier_task.c` / `rectifier_task.h`

5 ms (200 Hz) tick, `osPriorityAboveNormal` (4), 1 KB stack.

- **`clamp_i16(v, lo, hi)`** *(static inline)*.
- **`rectifier_task_start()`** — creates a static 8-deep RX queue,
  `can_mgr_subscribe(CAN_BUS_POWERTRAIN, 0x201, 0x7FF, ext=false)`, then
  creates the task.
- **`rectifier_task`** *(static)* loop, every 5 ms:
  1. Drain RX queue → `vesc_proto_decode_rect_state_concise` → on OK,
     write `g_pt.rect_state` and `rect_state_tick`, clear
     `FAULT_RECT_STALE`.
  2. Snapshot `g_pt.I_rect_cmd_cA` and `mode`, clamp to
     ±`I_RECT_MAX_CA`, build a `vesc_curr_dem_t` with local seq counter.
  3. `vesc_proto_encode_curr_dem` → `can_mgr_send` to CAN2.
  4. If `rect_state_tick` is set and older than 200 ms → set
     `FAULT_RECT_STALE`.

### `pdb_task.c` / `pdb_task.h`

20 ms tick, `osPriorityAboveNormal` (4), 1 KB stack. Failsafe-critical.

- **`pdb_task_start()`** — creates the task.
- **`pdb_task`** *(static)* loop, every 20 ms:
  1. Snapshot `supervisor_heartbeat` and contactor commands.
  2. If heartbeat changed → mark supervisor alive, clear
     `FAULT_SUPERVISOR_HANG`.
  3. Else if 100 ms passed without progress → mark supervisor hung,
     set `FAULT_SUPERVISOR_HANG`. **Stop writing GPIOs** (hold-last-state).
  4. If supervisor alive → `pdb_hw_write(bat_cmd, rect_cmd)`.
  Mid-flight contactor opens are **never automatic** — a load-bearing
  invariant per memory.

### `sensor_task.c` / `sensor_task.h`

10 ms tick, `osPriorityNormal` (3), 1.5 KB stack.

- **`sensor_task_start()`** — creates the data mutex, calls `ADC_Init`
  with a default 4-channel unity-gain config, then creates the task.
- **`sensor_data_get(*out)`** — mutex-protected snapshot of the latest
  readings (full struct copy).
- **`sensor_task`** *(static)* loop, every 10 ms:
  1. If `ADC_DataReady()` → `ADC_ReadAndStore` → if scan done,
     update `s_data.adc`.
  2. For each of 3 thermocouples: `MAX_ReadSPI`, `MAX_GetData`, store
     reading + valid flag.
  3. Update `s_data.tick`.
- **`HAL_GPIO_EXTI_Callback(pin)`** — overrides the HAL weak. On
  `ADC_DRDY_Pin`, calls `ADC_DRDY_IRQ()`.

### `log_task.c` / `log_task.h`

100 ms (10 Hz) tick, `osPriorityLow` (2), 2 KB stack. Only task that
touches FatFs.

- **`pick_filename`** *(static)* — finds first unused `LOG0000.CSV` ..
  `LOG9999.CSV`, falls back to `LOG.CSV` if all used.
- **`log_task_start()`** — creates the task.
- **`log_task`** *(static)* — first iteration: `f_mount`, pick filename,
  `f_open` with `FA_CREATE_ALWAYS`, write CSV header. Then loop every
  100 ms:
  1. Mutex-snapshot `g_pt`.
  2. `sensor_data_get` snapshot.
  3. `snprintf` 17-column row (TC temps as int32 cdeg ×100, ADC as raw).
  4. `f_write`; every 10 writes call `f_sync`.
  On any FatFs error during init, falls into a quiet `osDelay(1000)`
  loop (log is non-essential, must not block).

### `fc_link_task.c` / `fc_link_task.h`

50 ms tick, `osPriorityAboveNormal` (4), 2 KB stack. Sole owner of
`CanardInstance`. Wrapped in `__has_include` guards — no-op stub when
`canard.h` and the generated codecs aren't present.

- **`fc_link_task_start()`** — creates the static 32-deep RX queue,
  `can_mgr_subscribe(CAN_BUS_AVIONICS, 0, 0, ext=true)` (match-all),
  `canardInit` over a 2 KB static memory pool,
  `canardSetLocalNodeID(50)`, then creates the task.
- **`should_accept`** *(static)* — libcanard filter callback. Accepts
  broadcast `THROTTLEDEMAND` and `NODESTATUS`, fills the 64-bit
  signature, rejects everything else.
- **`on_transfer_received`** *(static)* — libcanard delivery callback.
  - On `THROTTLEDEMAND`: `dsdl_lat_powertrain_ThrottleDemand_decode` →
    `pt_set_fc_inputs(mode, throttle_pct × 100)`.
  - On `NODESTATUS` (from FC): timestamp `s_fc_last_status_tick`,
    clear `FAULT_FC_STALE`.
- **`publish_pt_concise`** *(static)* — snapshot `g_pt`, build a
  `PTConcise` (placeholder values for `i_load`/`i_bat`/
  `fuel_consumption`), encode, `canardBroadcast` at priority MEDIUM.
- **`publish_node_status(uptime)`** *(static)* — broadcast our 1 Hz
  `NodeStatus` with `pt_get_faults()` as `vendor_specific_status_code`.
- **`drain_canard_tx`** *(static)* — peek libcanard's TX queue, copy
  into `can_frame_t`, `can_mgr_send` to CAN1, pop on success. Bail if
  the manager queue is full — retry next tick.
- **`fc_link_task`** *(static)* loop, every 50 ms:
  1. Drain CAN1 RX queue → `canardHandleRxFrame` (timestamp in µs).
  2. PTConcise every 50 ms; NodeStatus every 1000 ms.
  3. Set `FAULT_FC_STALE` if last FC NodeStatus > 1 s old.
  4. `drain_canard_tx`.
  5. `canardCleanupStaleTransfers`.

### `bms_task.c` / `bms_task.h`

100 ms tick, `osPriorityNormal` (3), 1 KB stack. **Placeholder.**

- **`bms_task_start()`** — creates 8-deep RX queue, match-all subscribe
  on `CAN_BUS_POWERTRAIN` (will move to Bus 3 on H7), creates the task.
- **`parse_bms_frame(*f)`** *(static)* — **stub returning false.**
  TODO: decode protocol, write `g_pt.bms_*`, stamp `bms_input_tick`,
  return `true`.
- **`bms_task`** *(static)* loop, every 100 ms:
  1. Drain RX → `parse_bms_frame` → on `true`, clear `FAULT_BMS_STALE`.
  2. If `bms_input_tick` is set and older than 1 s → set
     `FAULT_BMS_STALE`. Stale only arms after first parse.

### `ecu_task.c` / `ecu_task.h`

100 ms tick, `osPriorityNormal` (3), 1 KB stack. **Placeholder, and
file header is now stale** — comments mention HFE/J1939 but the actual
plan is Loweheiser MAVLink over UART (see
`memory/project_ecu_loweheiser_mavlink.md`). Task must be refactored to
a UART RX handler.

- **`ecu_task_start()`** — match-all subscribe on `CAN_BUS_POWERTRAIN`
  (wrong long-term — will move to UART). Creates task.
- **`parse_ecu_frame(*f)`** *(static)* — stub returning false.
- **`ecu_task`** *(static)* — same shape as `bms_task`.

---

## 6. DSDL & libcanard

### `dsdl/lat/powertrain/20100.PTConcise.uavcan`

Broadcast PCU → FC at 20 Hz. DTID 20100. Fields: `egu_ok`, `batt_ok`,
6× float16 (`v_bus`, `i_load`, `i_bat`, `i_rect`, `batt_soc`,
`fuel_consumption`).

### `dsdl/lat/powertrain/20101.ThrottleDemand.uavcan`

Broadcast FC → PCU at 50 Hz. DTID 20101. `flight_phase` 4-bit enum
(matches `vesc_mode_t`), `throttle_pct` float16.

### `Middlewares/Third_Party/libcanard/`

Three files: `canard.c`, `canard.h`, `canard_internals.h`. Vendored
from [dronecan/libcanard](https://github.com/dronecan/libcanard) (v0
DroneCAN-flavored API). Public functions used: `canardInit`,
`canardSetLocalNodeID`, `canardHandleRxFrame`, `canardPeekTxQueue`,
`canardPopTxQueue`, `canardBroadcast`, `canardCleanupStaleTransfers`.

### `Middlewares/Third_Party/dronecan_codec/`

Auto-generated by `dronecan_dsdlc` from `dsdl/` + the standard DroneCAN
DSDL repo. Layout:
- `include/<dotted.name>.h`, `src/<dotted.name>.c` (one pair per
  message).
- Symbols: `dsdl_lat_powertrain_*` for our messages,
  `uavcan_protocol_NodeStatus` etc. for the standard ones.
- Encode signature: `uint32_t enc(struct *, uint8_t *)` returning
  byte length.
- Decode signature: `bool dec(const CanardRxTransfer *, struct *)`
  returning **`false` on success**, `true` on failure.
- Regenerate when DSDL changes; CMake `GLOB_RECURSE` re-globs each
  build (`CONFIGURE_DEPENDS`), so no manual list to maintain.

---

## 7. Build

### `CMakeLists.txt`

Top-level. Auto-globs `Core/Src/*.c`, `MIDWARE/*/*.c`,
`libcanard/*.c`, recursive on `dronecan_codec/*.c`. Subtracts the
CubeMX-managed source list (also in `cmake/stm32cubemx/CMakeLists.txt`)
to avoid duplicate-symbol errors. Includes:
- `MIDWARE/FATFS_SD`
- `Middlewares/Third_Party/libcanard`
- `Middlewares/Third_Party/dronecan_codec/include`

### `cmake/stm32cubemx/CMakeLists.txt`

CubeMX-owned source list and include dirs. Don't hand-edit.

### `Software_Hybrid.ioc`

CubeMX project file. Single source of truth for clocks, peripherals,
FreeRTOS config, pinout. Critical settings:
- `FREERTOS.configUSE_TICK_HOOK = 1` (required for FATFS_SD timers).
- `FREERTOS.configCHECK_FOR_STACK_OVERFLOW = 2`.
- `FREERTOS.configUSE_MALLOC_FAILED_HOOK = 1`.
- `IWDG` enabled; CAN1 at 1 Mbps; CAN2 at 250 kbps.

---

## 8. Runtime Topology

```
                       (CAN1, 1 Mbps DroneCAN)
                                │
                          ┌─────┴─────┐
                          │           │
              ThrottleDemand 50 Hz, NodeStatus 1 Hz
                          │           │
                          ▼           ▲
                   ┌──────────────────────┐
                   │   fc_link_task       │   ← libcanard, 50 ms tick
                   │   (sole CanardInstance)
                   └────────┬─────────────┘
                            │ pt_set_fc_inputs
                            ▼
               ┌──────────────────────────┐
               │   g_pt (mutex-protected) │
               └──────────────────────────┘
                  ▲                  │
                  │ pt_set_setpoint  │ snapshot
                  │                  ▼
            ┌─────┴────────┐    ┌────────────────┐
            │ supervisor   │    │ rectifier_task │   5 ms / 200 Hz
            │ 10 ms / 5 prio│    │ encode SendCurrDem │
            └──────┬───────┘    └────────┬───────┘
                   │ heartbeat            │ CAN2 0x101
                   ▼                      ▼
            ┌──────────────┐       (engine bus, 250 kbps)
            │  pdb_task    │             │
            │  20 ms / 4   │             ▲
            └──────┬───────┘       0x201 GetRectStateConcise
                   │ pdb_hw_write        │
                   ▼                  ┌──┴────────┐
              [PB11/PC15]              │   VESC    │
              (contactors)             │  (custom  │
                                       │   firmware│
                                       │   needed) │
                                       └───────────┘

  sensor_task (10 ms) ──► sensor_data ──► log_task (100 ms) ──► SD card
       │
       ├─ ADS1262 SPI3
       └─ 3× MAX31855 SPI2

  bms_task (100 ms, placeholder) ──► g_pt.bms_* (TODO parser)
  ecu_task (100 ms, placeholder, will become UART) ──► g_pt.ecu_* (TODO)
```

---

## 9. What's Remaining

### Tier 1 — done

- ~~IWDG~~ ✓ (64 ms timeout)
- ~~PDB pins in `.ioc`~~ ✓ (PB11/PC15)
- ~~Stack overflow + malloc-failed hooks~~ ✓
- ~~Bus-off auto-recovery~~ ✓ (still no detection — Tier 3)
- VESC firmware modifications — separate workstream

### Tier 2 — partial

- ~~SWO/ITM `printf`~~ ✓
- ~~LED heartbeat~~ ✓
- ADS1262 front-end scaling — deferred; user not yet sure of channel
  assignments
- BMS protocol parsing — placeholder ready; protocol unknown
- ECU protocol — placeholder is **wrong-bus** (CAN); needs UART +
  Loweheiser MAVLink refactor (see memory)
- `fc_link_task::publish_pt_concise` `i_load`/`i_bat`/
  `fuel_consumption` placeholders — fill once ADC scaling, BMS, ECU
  are real
- EGU_OK / Batt_OK derivation — currently placeholder

### Tier 3 — hardening

- `FAULT_BUS1_BUSOFF` / `FAULT_BUS2_BUSOFF` detector
  (`HAL_CAN_ErrorCallback`)
- Filter coalescing in `can_manager` (today: rectifier + bms + ecu +
  fc_link + can_callbacks-driven hardware = several banks consumed)
- Replace `rx_dispatch_task` `osDelay(1)` poll with thread flags
- CPU load metering from idle hook (count idle ticks, expose in `g_pt`)
- Boot hardware self-tests (ADS1262 ID, MAX31855 fault check, SD mount)
- Fault-clear policy review (which bits self-clear vs sticky)
- `Error_Handler` LED pattern (slow blink = expected, fast = unexpected)
- Stale `ecu_task.c` header comment ("HFE/J1939") — needs rewrite for
  Loweheiser MAVLink

### Tier 4 — port & verification

- F4 → H7 port (bxCAN → FDCAN, BMS bus reassignment via
  `#define BMS_BUS`)
- CAN2 baud confirmation against ECU/BMS max bitrates
- CAN1 1 Mbps confirmation against Pixhawk
- ECU MAVLink refactor (UART + Loweheiser dialect)
- Bench bring-up sequence: sniff CAN2, then VESC RX-only, then
  closed-loop low-current
