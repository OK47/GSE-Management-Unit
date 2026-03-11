#include "KJO_QR_Slave.h"

//
// QR_Slave — GSEMU-side umbilical quick-release actuator.
// Level-based command following with physical separation detection.
// See KJO_QR_Slave.h for full protocol description.
//

// --- Constructor -------------------------------------------------------------
QR_Slave::QR_Slave( byte PWM_channel, int PWM_hold, int PWM_open,
                    uint16_t move_time_ms, Adafruit_PWMServoDriver *Servos,
                    Adafruit_MCP23X17 *gpio, byte cmd_pin, byte state_pin )
    : QR_Servo( PWM_channel, PWM_hold, PWM_open, move_time_ms, Servos ),
      _gpio( gpio ), _cmd_pin( cmd_pin ), _state_pin( state_pin ),
      _prev_state( LOW ), _separated( false ), _local_override( false )
{
}

// --- begin() -----------------------------------------------------------------
//
// Configures the CMD and state pins, reads the initial state of the state line,
// and commands the servo to HOLD (latch engaged).
//
void QR_Slave::begin()
{
    // CMD pin: INPUT_PULLUP — receives level command from EMU.
    _gpio->pinMode( _cmd_pin,   INPUT_PULLUP );

    // State pin: INPUT_PULLUP — wired to EMU chassis GND.
    // Reads LOW when umbilical is connected, HIGH when separated.
    _gpio->pinMode( _state_pin, INPUT_PULLUP );

    // Capture the initial state line value (expected LOW = connected at startup).
    _prev_state = _gpio->digitalRead( _state_pin );
    _separated  = ( _prev_state == HIGH );  // already separated if HIGH at startup

    // Command servo to hold (latch engaged).
    QR_Servo::begin();
}

// --- update() ----------------------------------------------------------------
//
// Must be called from loop() on every iteration.
//
// Servo control (level-based):
//   CMD LOW  + servo not already opening or open  → openServo()
//   CMD HIGH + servo not already holding or held  → holdServo()
//
// Separation detection:
//   Monitors the state line for a LOW → HIGH transition.
//   Sets the _separated latch on first confirmed HIGH.
//
void QR_Slave::update()
{
    // Update the servo's timed-move state machine.
    isMoving();

    // ── Servo command following ───────────────────────────────────────────────
    // Local override (Button A held) takes priority over the CMD line.
    // While _local_override is true the CMD line is ignored and the servo is
    // commanded open regardless of what the EMU is driving.
    bool cmd = _local_override ? LOW : _gpio->digitalRead( _cmd_pin );

    if( cmd == LOW )
    {
        // CMD LOW: EMU commands latch open.
        // Only issue the command if we are not already opening or fully open.
        if( _servo_state != QR_Servo_State::MOVING_TO_OPEN &&
            _servo_state != QR_Servo_State::AT_OPEN )
        {
            openServo();
        }
    }
    else
    {
        // CMD HIGH (or disconnected pullup): hold latch.
        // Only issue the command if we are not already holding or fully held.
        if( _servo_state != QR_Servo_State::MOVING_TO_HOLD &&
            _servo_state != QR_Servo_State::AT_HOLD )
        {
            holdServo();
        }
    }

    // ── Physical separation detection ────────────────────────────────────────
    bool curr_state = _gpio->digitalRead( _state_pin );

    if( curr_state == HIGH && _prev_state == LOW )
    {
        // Rising edge on state line: umbilical has physically separated.
        _separated = true;
    }

    _prev_state = curr_state;
}

// --- State accessors ---------------------------------------------------------

// Returns true once the state line has gone HIGH (physical separation confirmed).
// Latched — returns true for the remainder of the session after separation.
bool QR_Slave::isSeparated() { return _separated; }

// Returns true while the state line is LOW (umbilical physically connected).
bool QR_Slave::isConnected() { return ( _gpio->digitalRead( _state_pin ) == LOW ); }

// --- Local override ----------------------------------------------------------

// Engage local release: servo opens on the next update() regardless of CMD line.
void QR_Slave::localRelease()
{
    _local_override = true;
}

// Release local override: servo returns to following the CMD line on the next update().
void QR_Slave::localHold()
{
    _local_override = false;
}

// Returns true while the local button override is active.
bool QR_Slave::isLocalOverride() { return _local_override; }
