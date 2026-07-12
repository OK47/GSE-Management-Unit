# GSEMU CAN Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give GSEMU a CAN link to EMU/LCMU, retire the wired Fill-valve GPIO handshake, and make GSEMU autonomously drive the Fill valve to a target weight reported by LCMU, closing it itself once the target is reached — rather than waiting for an EMU-issued close.

**Architecture:** GSEMU gains its first CAN participation ever (it currently has no radio link of any kind — it's purely GPIO-wired to EMU). An `Adafruit_MCP2515` instance is polled from `loop()` (no INT pin, matching the EMU plan). The wired `Slave_Valve`/`Fill_Valve` object is replaced with a plain `Valve` (the wired command/status handshake it existed to service is retired; the base `Valve` class's `open()`/`close()`/`isOpen()`/`isClosed()` are already called directly elsewhere in this file for the front-panel test buttons, so this is a drop-in swap, not a new pattern). One of the two pins freed by retiring the handshake is repurposed as GSEMU's own ground-referenced link-disconnect sense line, mirroring EMU's `RELEASE_STATE_EIO` technique exactly (this codebase's existing convention for umbilical-disconnect detection). A new `Fill_State_Machine` module — armed by EMU's `CAN_BEGIN_FILL`, tears down on `CAN_ABORT_FILL` — tares LCMU, opens the valve, polls LCMU's weight against the stored target, and closes the valve autonomously when reached, all non-blocking from `loop()`.

**Tech Stack:** PlatformIO, Arduino framework, atmelsam platform (unchanged), Adafruit_MCP2515, Adafruit_CAN (already in `platformio.ini`), `KJO_CAN_Command_Defs.h` (from the LCMU core firmware plan, `KJO_Shared_Libraries`).

## Global Constraints

- CAN bus: 250 kbps, MCP2515 crystal 16 MHz (library default).
- Node IDs: EMU=1, GSEMU=2, LCMU=3. `CAN_ID = (destination << 4) | source` (`KJO_CAN_Command_Defs.h`).
- Polling only, no INT pin needed (same rationale as the EMU plan — `parsePacket()` from `loop()`, `onReceive()` never called).
- `CAN_CS_PIN` is a concrete starting value in this plan (see Task 1), subject to confirmation once GSEMU's CAN FeatherWing is physically wired — GSEMU's pin budget is less constrained than EMU's per prior discussion, but the exact pin is still hardware-dependent.
- Retiring the wired Fill-valve handshake frees `FILL_VALVE_CMD_EIO` (MCP23X17 AUX_IO_1) and `FILL_VALVE_STATUS_EIO` (AUX_IO_2) in GSEMU's own `KJO_GPIO.h` (a separate file from EMU's, not shared). This plan repurposes AUX_IO_1 as `GSEMU_LINK_SENSE_EIO`, wired to EMU chassis GND — the same technique already used by `RELEASE_STATE_EIO` (`KJO_GPIO.h:53-57`). AUX_IO_2 is left unused.
- Fill target units: lbm, carried over CAN as `CAN_Command_Frame.param` (a `float`, no scaling needed — unlike the LoRa `Command_Payload.param` used between RCU/EMU, the CAN frame's param field is already a native float).
- Weight-reached comparison: GSEMU polls LCMU's `CAN_REPORT_CURRENT_WEIGHT` and compares the returned value against the stored target directly (no tolerance/hysteresis band — the spec calls the resulting overshoot from CAN-speed polling "vanishingly small" against the fill quantities involved).
- All existing GSEMU behavior not touched by this plan (QR release, battery display, AUX input monitoring, buzzer) is unchanged.
- No hardware exists yet. Every task's verification is `pio run` (compiles clean).
- `CAN_GET_HEALTH_STATUS` (added to `KJO_CAN_Command_Defs.h` during the LCMU plan's final-review fix) lets any CAN node query any other node's fault status — a bitmask decimal-encoded into `CAN_Response_Frame.r_val`. Task 6 implements GSEMU's response.

---

### Task 1: CAN hardware bring-up; retire Slave_Valve; repurpose link-sense pin

**Files:**
- Modify: `GSE Management Unit/src/main.cpp`
- Modify: `GSE Management Unit/src/KJO_GPIO.h`

**Interfaces:**
- Consumes: `KJO_CAN_Command_Defs.h` (LCMU core firmware plan, Task 1).
- Produces: `CAN_Controller` (global `Adafruit_MCP2515`), `GSEMU_LINK_SENSE_EIO`, `Fill_Valve` as a plain `Valve` — consumed by Task 2 (CAN helper) and Task 4 (fill state machine).

- [ ] **Step 1: Repurpose the freed Fill-valve handshake pin in KJO_GPIO.h**

```cpp
// GSE Management Unit/src/KJO_GPIO.h -- replace the "AUX 1: Fill valve
// handshake" block (lines 31-43) with:

// --- AUX 1: GSEMU<->EMU link-disconnect sense --------------------------------
// The wired Fill-valve command/status handshake once carried on these two
// pins has been retired in favor of CAN-orchestrated fill control (see
// docs/superpowers/specs/2026-07-11-lcmu-design.md). AUX_IO_1 is repurposed
// as a link-disconnect sense line, using the same ground-reference
// technique already used by RELEASE_STATE_EIO below: wired to EMU chassis
// GND, so a physical disconnect is detected instantly and independently of
// any CAN bus traffic or timeout logic.
//
// I/O 1 — GSEMU<->EMU link sense (GSEMU INPUT_PULLUP <- EMU GND):
//   LOW  = umbilical connector is physically connected.
//   HIGH = umbilical connector has separated (INPUT_PULLUP floats HIGH).
constexpr uint8_t GSEMU_LINK_SENSE_EIO  = AUX_IO_1_EIO;

// AUX_IO_2 (formerly FILL_VALVE_STATUS_EIO) is currently unused.
```

- [ ] **Step 2: Replace Slave_Valve with a plain Valve for the Fill valve**

```cpp
// GSE Management Unit/src/main.cpp -- delete this line entirely
// (KJO_Valve.h is already included separately a few lines above it, so
// no replacement include is needed):
#include "KJO_Slave_Valve.h"        // GSEMU-side servo valve driven by EMU GPIO handshake

// Replace the Fill_Valve declaration:
// Fill valve  --  servo-actuated, driven by EMU GPIO handshake via Slave_Valve.
// Hardware constants from KJO_Valve.h; GPIO pins from KJO_GPIO.h.
Slave_Valve Fill_Valve( FILL_VALVE,
                        FILL_VALVE_PWM, FILL_VALVE_PWM_OPEN, FILL_VALVE_PWM_CLOSE,
                        FILL_VALVE_ANALOG_PIN, FILL_VALVE_POS_OPEN, FILL_VALVE_POS_CLOSED,
                        FILL_VALVE_BALL_DIAMETER, FILL_VALVE_BORE_DIAMETER, FILL_VALVE_THROAT_DIAMETER,
                        &Servos,
                        &E_GPIO, FILL_VALVE_CMD_EIO, FILL_VALVE_STATUS_EIO );
// with:
// Fill valve  --  servo-actuated, now driven autonomously by GSEMU's own
// CAN-orchestrated fill state machine (Task 4) rather than an EMU GPIO
// handshake. Hardware constants from KJO_Valve.h.
Valve Fill_Valve( FILL_VALVE,
                  FILL_VALVE_PWM, FILL_VALVE_PWM_OPEN, FILL_VALVE_PWM_CLOSE,
                  FILL_VALVE_ANALOG_PIN, FILL_VALVE_POS_OPEN, FILL_VALVE_POS_CLOSED,
                  FILL_VALVE_BALL_DIAMETER, FILL_VALVE_BORE_DIAMETER, FILL_VALVE_THROAT_DIAMETER,
                  &Servos );
```

- [ ] **Step 3: Remove the retired Fill_Valve.update() call from loop()**

```cpp
// GSE Management Unit/src/main.cpp -- in loop(), remove:
    // Service Fill valve: check command bit and update servo + status bit
    Fill_Valve.update();
```

Note: plain `Valve` has no `update()` method — motion state is polled
directly via `isMoving()`/`isOpen()`/`isClosed()`, which is how the
existing front-panel test buttons in `Check_Buttons()` already use it, so
no replacement call is needed here.

- [ ] **Step 4: Add CAN init and the link-sense pin to setup()**

```cpp
// GSE Management Unit/src/main.cpp -- add near the top, with other includes
#include <Adafruit_MCP2515.h>
#include "KJO_CAN_Command_Defs.h"

// CS pin: this plan's concrete starting value, subject to confirmation
// once the CAN FeatherWing is physically wired -- see Global Constraints.
// No INT pin is needed: this design polls parsePacket() from loop().
constexpr uint8_t CAN_CS_PIN = 11;

Adafruit_MCP2515 CAN_Controller( CAN_CS_PIN );

// GSE Management Unit/src/main.cpp -- in setup(), replacing the old
// "Fill valve (Slave_Valve: closes to known state, configures GPIO pins)" block:
    // -- Fill valve (plain Valve: closes to known state) ------------------------
    Fill_Valve.begin();
    scrollMessage( &Screen, "Fill valve ready.", true );

    // -- Link-disconnect sense (repurposed from the old Fill-valve handshake) --
    E_GPIO.pinMode( GSEMU_LINK_SENSE_EIO, INPUT_PULLUP );

    // -- CAN bus -----------------------------------------------------------
    if( !CAN_Controller.begin( CAN_BAUDRATE ) )
    {
        Post_Log_Message( "[CAN] mcp.begin() FAILED" );
        scrollMessage( &Screen, "CAN init FAILED", true );
    }
    else
    {
        Post_Log_Message( "[CAN] ready" );
        scrollMessage( &Screen, "CAN ready.", true );
    }
```

- [ ] **Step 5: Verify it compiles**

Run: `cd "GSE Management Unit" && pio run`
Expected: `SUCCESS`

- [ ] **Step 6: Commit**

```bash
git add src/main.cpp src/KJO_GPIO.h
git commit -m "Add CAN bring-up; replace Slave_Valve with plain Valve; repurpose link-sense pin"
```

---

### Task 2: CAN command send-and-wait helper (GSEMU -> LCMU)

**Files:**
- Modify: `GSE Management Unit/src/main.cpp`

**Interfaces:**
- Consumes: `CAN_Controller` (Task 1), `CAN_Command_Frame`/`CAN_Response_Frame`/`CAN_Pack_ID`/`CAN_Unpack_Destination`/`CAN_Unpack_Source` (`KJO_CAN_Command_Defs.h`).
- Produces: `Send_CAN_Command( uint8_t destination, CAN_Command command, float param, CAN_Response_Frame &response_out )` returning `bool` — consumed by Task 4 (fill state machine) and Task 3 (relaying EMU's queries).

**Correction (post-Task-4-review, user decision):** `CAN_RESPONSE_TIMEOUT_MS` is raised from 200ms to 500ms. This value doubles as the per-attempt timeout for Task 4's LCMU weight-poll retries (see Task 4's correction) — the user specified 500ms as the reasonable per-attempt window before counting a poll as a miss.

- [ ] **Step 1: Add the helper to main.cpp**

```cpp
// GSE Management Unit/src/main.cpp -- add below the CAN_Controller declaration
constexpr unsigned long CAN_RESPONSE_TIMEOUT_MS = 500;   // bumped from 200ms -- see Task 4 correction

// Sends a CAN command frame to 'destination' and blocks (bounded by
// CAN_RESPONSE_TIMEOUT_MS) waiting for its response. Returns false on
// timeout (response_out is left untouched).
bool Send_CAN_Command( uint8_t destination, CAN_Command command, float param,
                        CAN_Response_Frame &response_out )
{
    CAN_Command_Frame frame;
    frame.command = command;
    frame.param   = param;

    CAN_Controller.beginPacket( CAN_Pack_ID( destination, CAN_NODE_GSEMU ) );
    CAN_Controller.write( (uint8_t *)&frame, sizeof( frame ) );
    CAN_Controller.endPacket();

    unsigned long start = millis();
    while( millis() - start < CAN_RESPONSE_TIMEOUT_MS )
    {
        int packet_size = CAN_Controller.parsePacket();
        if( packet_size <= 0 ) continue;

        uint32_t id = CAN_Controller.packetId();
        if( CAN_Unpack_Destination( id ) != CAN_NODE_GSEMU ) continue;
        if( CAN_Unpack_Source( id )      != destination )     continue;

        CAN_Controller.readBytes( (uint8_t *)&response_out, sizeof( response_out ) );
        if( response_out.command != command ) continue;   // stray/unrelated response

        return true;
    }

    return false;
}
```

- [ ] **Step 2: Verify it compiles**

Run: `pio run`
Expected: `SUCCESS`

- [ ] **Step 3: Commit**

```bash
git add src/main.cpp
git commit -m "Add Send_CAN_Command helper for GSEMU -> LCMU requests"
```

---

### Task 3: CAN command dispatch (EMU -> GSEMU)

**Files:**
- Modify: `GSE Management Unit/src/main.cpp`

**Interfaces:**
- Consumes: `CAN_Controller` (Task 1), `Send_CAN_Command()` (Task 2).
- Produces: `Check_CAN()` — called from `loop()`. Dispatches `CAN_SET_FILL_TARGET`, `CAN_BEGIN_FILL`, `CAN_QUERY_FILL_STATUS`, `CAN_ABORT_FILL` (all four addressed to GSEMU per the spec's command table). Forward-declares the fill-state-machine entry points defined in Task 4.

**Correction (post-Task-4-review, user decision):** `Fill_Is_Complete()` (returning `bool`) is replaced by `Fill_Get_Status()` (returning the shared `Fill_Status` enum — `KJO_Shared_Libraries/KJO_CAN_Command_Defs.h`), so EMU can distinguish "target reached" from "gave up after repeated LCMU comms failures" instead of seeing both as the same signal. The `CAN_QUERY_FILL_STATUS` case below is updated to send the 3-state code in `r_val` instead of the bool in `status`.

- [ ] **Step 1: Add the CAN dispatch loop**

```cpp
// GSE Management Unit/src/main.cpp -- add below the Send_CAN_Command helper

// Defined in Task 4 (fill state machine).
void  Fill_Set_Target( float target_lbm );
bool  Fill_Begin();
void  Fill_Abort();
bool  Fill_Is_Complete();

static void Send_CAN_Response( uint8_t destination, CAN_Command command, bool status, float r_val )
{
    CAN_Response_Frame resp;
    resp.command = command;
    resp.status  = status;
    resp.r_val   = r_val;

    CAN_Controller.beginPacket( CAN_Pack_ID( destination, CAN_NODE_GSEMU ) );
    CAN_Controller.write( (uint8_t *)&resp, sizeof( resp ) );
    CAN_Controller.endPacket();
}

void Check_CAN()
{
    int packet_size = CAN_Controller.parsePacket();
    if( packet_size <= 0 ) return;

    uint32_t id          = CAN_Controller.packetId();
    uint8_t  destination = CAN_Unpack_Destination( id );
    uint8_t  source      = CAN_Unpack_Source( id );

    if( destination != CAN_NODE_GSEMU ) return;   // not addressed to us

    CAN_Command_Frame frame;
    CAN_Controller.readBytes( (uint8_t *)&frame, sizeof( frame ) );

    switch( frame.command )
    {
        case CAN_SET_FILL_TARGET:
            Fill_Set_Target( frame.param );
            Send_CAN_Response( source, CAN_SET_FILL_TARGET, true, 0.0f );
            break;

        case CAN_BEGIN_FILL:
            Send_CAN_Response( source, CAN_BEGIN_FILL, Fill_Begin(), 0.0f );
            break;

        case CAN_QUERY_FILL_STATUS:
            Send_CAN_Response( source, CAN_QUERY_FILL_STATUS, true, (float)Fill_Get_Status() );
            break;

        case CAN_ABORT_FILL:
            Fill_Abort();
            Send_CAN_Response( source, CAN_ABORT_FILL, true, 0.0f );
            break;

        default:
            // TARE / GET_BATTERY_VOLTAGE / REPORT_CURRENT_WEIGHT /
            // BEGIN-STOP_WEIGHT_RECORDING / BEGIN-STOP_THRUST_RECORDING are
            // LCMU-side commands -- not handled by GSEMU.
            break;
    }
}
```

- [ ] **Step 2: Call Check_CAN() from loop()**

```cpp
// GSE Management Unit/src/main.cpp -- in loop(), add:
    Check_CAN();
```

- [ ] **Step 3: Verify it compiles (deferred — Task 4 defines the forward-declared functions)**

The forward-declared `Fill_Set_Target()`/`Fill_Begin()`/`Fill_Abort()`/
`Fill_Is_Complete()` have no definitions yet, so this task alone will not
link. Compile verification happens at the end of Task 4.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "Add CAN command dispatch for SET_FILL_TARGET/BEGIN_FILL/QUERY_FILL_STATUS/ABORT_FILL"
```

---

### Task 4: Fill state machine (tare, open, poll LCMU weight, autonomous close)

**Files:**
- Modify: `GSE Management Unit/src/main.cpp`

**Interfaces:**
- Consumes: `Send_CAN_Command()` (Task 2), `Fill_Valve` (Task 1: `open()`, `close()`, `isOpen()`, `isClosed()`).
- Produces: `Fill_Set_Target()`, `Fill_Begin()`, `Fill_Abort()`, `Fill_Get_Status()` (forward-declared in Task 3), `Check_Fill()` — called from `loop()`.

**Correction (post-review, user decision):** the original code below had two gaps found by Task 4's review: (1) no safety net if LCMU stops responding to weight polls once the valve is open — the valve could stay open indefinitely; (2) `Fill_Is_Complete()`'s bool couldn't distinguish "target reached" from "aborted." Both are fixed in the code below:
- `Fill_Is_Complete()` (bool) is replaced by `Fill_Get_Status()` (returns the shared `Fill_Status` enum: `FILL_STATUS_IN_PROGRESS` / `FILL_STATUS_COMPLETE` / `FILL_STATUS_ABORTED`).
- `Check_Fill()` now counts consecutive failed weight polls; after 3 in a row (`FILL_MAX_CONSECUTIVE_POLL_FAILURES`), it auto-aborts (closes the valve, marks `FILL_STATUS_ABORTED`) rather than retrying forever. Each poll attempt's own bounded wait comes from `Send_CAN_Command`'s `CAN_RESPONSE_TIMEOUT_MS`, raised to 500ms in Task 2's correction — so worst case this triggers roughly 1.5s after LCMU goes silent. This status flows to EMU via its existing `CAN_QUERY_FILL_STATUS` polling, and from there to the RCU operator via `QUERY_FILL_STATUS`/`READ_VALVE_POSITION` (see the EMU CAN-integration plan's matching correction).

```cpp
// GSE Management Unit/src/main.cpp -- add below the Check_CAN() block

// --- Fill state machine ---------------------------------------------------------
// Armed by EMU's CAN_BEGIN_FILL (after CAN_SET_FILL_TARGET has already
// stored the target). Runs entirely locally on GSEMU: tares LCMU, opens
// the Fill valve, polls LCMU's current weight against the stored target,
// and closes the valve itself the moment the target is reached -- no
// further authorization from EMU is needed to close. If LCMU stops
// responding to weight polls, the fill is auto-aborted (valve closed)
// after FILL_MAX_CONSECUTIVE_POLL_FAILURES consecutive misses, rather
// than leaving the valve open indefinitely. EMU separately polls
// Fill_Get_Status() via CAN_QUERY_FILL_STATUS at a much lower rate.
constexpr unsigned long FILL_POLL_INTERVAL_MS               = 50;
constexpr uint8_t       FILL_MAX_CONSECUTIVE_POLL_FAILURES  = 3;

float         fill_target_lbm         = 0.0f;
bool          fill_active             = false;
Fill_Status   fill_state              = FILL_STATUS_IN_PROGRESS;
unsigned long last_fill_poll_ms       = 0;
uint8_t       fill_consecutive_fails  = 0;

void Fill_Set_Target( float target_lbm )
{
    fill_target_lbm = target_lbm;
}

bool Fill_Begin()
{
    if( fill_active ) return true;   // already running

    CAN_Response_Frame tare_resp;
    if( !Send_CAN_Command( CAN_NODE_LCMU, CAN_TARE, 0.0f, tare_resp ) )
    {
        Post_Log_Message( "[FILL] LCMU did not respond to tare -- fill not started." );
        return false;
    }

    Fill_Valve.open();
    fill_active            = true;
    fill_state              = FILL_STATUS_IN_PROGRESS;
    fill_consecutive_fails  = 0;
    last_fill_poll_ms       = millis();
    Post_Log_Message( String( "[FILL] Started -- target " ) + String( fill_target_lbm, 2 ) + " lbm" );
    return true;
}

void Fill_Abort()
{
    if( !fill_active ) return;

    Fill_Valve.close();
    fill_active = false;
    fill_state  = FILL_STATUS_ABORTED;
    Post_Log_Message( "[FILL] Aborted -- valve closed." );
}

Fill_Status Fill_Get_Status()
{
    return fill_state;
}

// Call every loop() iteration. Non-blocking: polls LCMU at
// FILL_POLL_INTERVAL_MS, closes the valve and marks complete once the
// target is reached. Auto-aborts after FILL_MAX_CONSECUTIVE_POLL_FAILURES
// consecutive missed polls rather than leaving the valve open forever.
void Check_Fill()
{
    if( !fill_active ) return;

    unsigned long now = millis();
    if( now - last_fill_poll_ms < FILL_POLL_INTERVAL_MS ) return;
    last_fill_poll_ms = now;

    CAN_Response_Frame weight_resp;
    if( !Send_CAN_Command( CAN_NODE_LCMU, CAN_REPORT_CURRENT_WEIGHT, 0.0f, weight_resp ) )
    {
        fill_consecutive_fails++;
        Post_Log_Message( String( "[FILL] LCMU did not respond to weight poll (" )
                         + String( fill_consecutive_fails ) + "/" + String( FILL_MAX_CONSECUTIVE_POLL_FAILURES ) + ")." );

        if( fill_consecutive_fails >= FILL_MAX_CONSECUTIVE_POLL_FAILURES )
        {
            Fill_Valve.close();
            fill_active = false;
            fill_state  = FILL_STATUS_ABORTED;
            Post_Log_Message( "[FILL] Aborted -- LCMU unreachable after 3 consecutive polls." );
        }
        return;   // try again next interval unless the threshold above just closed the valve
    }

    fill_consecutive_fails = 0;

    if( weight_resp.r_val >= fill_target_lbm )
    {
        Fill_Valve.close();
        fill_active = false;
        fill_state  = FILL_STATUS_COMPLETE;
        Post_Log_Message( String( "[FILL] Target reached at " ) + String( weight_resp.r_val, 2 )
                         + " lbm -- valve closed." );
    }
}
```

- [ ] **Step 2: Call Check_Fill() from loop()**

```cpp
// GSE Management Unit/src/main.cpp -- in loop(), add:
    Check_Fill();
```

- [ ] **Step 3: Verify it compiles**

Run: `pio run`
Expected: `SUCCESS`

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "Add autonomous fill state machine: tare, open, poll LCMU weight, close on target"
```

---

### Task 5: Final integration pass

**Files:**
- Modify: `GSE Management Unit/src/main.cpp`

**Interfaces:**
- Consumes: everything from Tasks 1-4.

- [ ] **Step 1: Confirm loop() ordering**

```cpp
// GSE Management Unit/src/main.cpp
void loop()
{
    // Service timed relay outputs
    Buzzer.update();

    // Service QR release servo: follow CMD level, detect physical separation
    QR_Release.update();

    // Log umbilical separation event once when first confirmed
    if( QR_Release.isSeparated() && !qr_sep_logged )
    {
        Post_Log_Message( "QR: umbilical separated." );
        scrollMessage( &Screen, "QR: Released", false );
        qr_sep_logged = true;
    }

    // Check front-panel buttons (local control)
    Check_Buttons();

    // Display battery voltage every BATTERY_DISPLAY_INTERVAL_MS (non-blocking)
    Check_Battery();

    // Poll AUX analog input; drive inverted output and log on state change
    Check_AUX_Input();

    // CAN command dispatch (EMU -> GSEMU) and fill state machine
    Check_CAN();
    Check_Fill();
}
```

Note: `Fill_Valve.update()` (originally called here) has already been
removed in Task 1, Step 3.

- [ ] **Step 2: Delete the now-unused KJO_Slave_Valve files**

`Slave_Valve` has no remaining callers anywhere in the project once Task 1
Step 2's swap to plain `Valve` is in place — delete it rather than leave
dead code behind:

```bash
git rm "GSE Management Unit/src/KJO_Slave_Valve.h" "GSE Management Unit/src/KJO_Slave_Valve.cpp"
```

- [ ] **Step 3: Confirm no remaining references to Slave_Valve**

Run: `grep -rn "Slave_Valve" "GSE Management Unit/src"`
Expected: no output.

- [ ] **Step 4: Verify the full build compiles**

Run: `pio run`
Expected: `SUCCESS`

- [ ] **Step 5: Commit**

```bash
git add src/main.cpp
git commit -m "Final integration pass for GSEMU CAN command handling; remove unused Slave_Valve"
```

---

### Task 6: CAN_GET_HEALTH_STATUS response

**Files:**
- Modify: `GSE Management Unit/src/main.cpp`

**Interfaces:**
- Consumes: `CAN_GET_HEALTH_STATUS`/`HEALTH_*` bit constants (`KJO_CAN_Command_Defs.h`, added during the LCMU plan's final-review fix), `Send_CAN_Response()` (Task 3), `GSEMU_LINK_SENSE_EIO` (Task 1).
- Produces: `Check_CAN()` handling for `CAN_GET_HEALTH_STATUS`. Consumed by EMU's `RCU_STATUS` handler (EMU CAN-integration plan, Task 8), which queries this to relay GSEMU's health up to RCU.

**Context:** Any CAN node can query any other node's fault status via `CAN_GET_HEALTH_STATUS`, added to the shared header so an operator using the RCU can diagnose a remote fault on a unit they can't physically approach. This task adds GSEMU's response: bits 0-2 (universal: SD init failure, RTC init failure, link-sense lost) plus bit 8 (`HEALTH_GSEMU_FILL_VALVE_FAIL`, reserved — always 0 for now, since this plan doesn't implement Fill-valve fault detection).

- [ ] **Step 1: Track SD/RTC init status**

```cpp
// GSE Management Unit/src/main.cpp -- near the top, with other globals:
bool sd_ok  = false;   // set in setup(); used for CAN_GET_HEALTH_STATUS
bool rtc_ok = false;   // set in setup(); used for CAN_GET_HEALTH_STATUS

// In setup(), wherever SD.begin(...) and RT_Clock.begin() are currently
// called, capture their return values into sd_ok/rtc_ok respectively
// (the existing code already branches on these calls for
// scrollMessage()/Post_Log_Message() purposes -- just also store the
// boolean rather than only branching on it inline).
```

- [ ] **Step 2: Add the CAN_GET_HEALTH_STATUS case to Check_CAN()**

```cpp
// GSE Management Unit/src/main.cpp -- add a case to the switch(frame.command)
// block in Check_CAN() (Task 3), alongside CAN_ABORT_FILL:

        case CAN_GET_HEALTH_STATUS:
        {
            uint16_t bits = 0;
            if( !sd_ok )                                        bits |= HEALTH_SD_INIT_FAIL;
            if( !rtc_ok )                                        bits |= HEALTH_RTC_INIT_FAIL;
            if( E_GPIO.digitalRead( GSEMU_LINK_SENSE_EIO ) == HIGH ) bits |= HEALTH_LINK_SENSE_LOST;
            // HEALTH_GSEMU_FILL_VALVE_FAIL intentionally always 0 -- no
            // fault detection implemented for the Fill valve in this plan.
            Send_CAN_Response( source, CAN_GET_HEALTH_STATUS, true, (float)bits );
            break;
        }
```

- [ ] **Step 3: Verify it compiles**

Run: `pio run`
Expected: `SUCCESS`

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "Add CAN_GET_HEALTH_STATUS response (SD/RTC/link-sense health)"
```

---

## Deferred to hardware bring-up (not part of this plan)

- Confirming/correcting `CAN_CS_PIN` once the CAN FeatherWing is
  physically wired to GSEMU.
- EMU CAN integration (separate plan) and LCMU core firmware (separate
  plan) — this plan assumes both exist and speak the same
  `KJO_CAN_Command_Defs.h` protocol.
- Whether `Fill_Abort()` should also trigger any GSE-side safing action
  beyond closing the valve (e.g. venting) — out of scope for this plan;
  flag to the user if a specific GSE safing sequence beyond "close the
  valve" is needed once hardware is available to test against.
