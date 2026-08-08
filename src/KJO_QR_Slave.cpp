#include "KJO_QR_Slave.h"

//
// QR_Slave — GSEMU-side umbilical quick-release actuator.
// One-shot CAN-commanded release with physical separation detection.
// See KJO_QR_Slave.h for full protocol description.
//

// --- Constructor -------------------------------------------------------------
QR_Slave::QR_Slave( byte PWM_channel, int PWM_hold, int PWM_open,
                    uint16_t move_time_ms, Adafruit_PWMServoDriver *Servos,
                    Adafruit_MCP23X17 *gpio, byte state_pin )
    : QR_Servo( PWM_channel, PWM_hold, PWM_open, move_time_ms, Servos ),
      _gpio( gpio ), _state_pin( state_pin ),
      _prev_state( LOW ), _separated( false ), _local_override( false ),
      _release_commanded( false )
{
}

// --- begin() -----------------------------------------------------------------
void QR_Slave::begin()
{
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
void QR_Slave::update()
{
    // Update the servo's timed-move state machine.
    isMoving();

    // ── Servo command following ───────────────────────────────────────────────
    // Local override (Button A held) or a latched release command both
    // mean "open." Neither is ever un-set by the other -- see
    // commandRelease()/localHold()'s doc comments in KJO_QR_Slave.h.
    bool release = _local_override || _release_commanded;

    if( release )
    {
        // Only issue the command if we are not already opening or fully open.
        if( _servo_state != QR_Servo_State::MOVING_TO_OPEN &&
            _servo_state != QR_Servo_State::AT_OPEN )
        {
            openServo();
        }
    }
    else
    {
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

// --- Release command (CAN) ----------------------------------------------------

// Permanent latch -- never cleared. See KJO_QR_Slave.h.
void QR_Slave::commandRelease()
{
    _release_commanded = true;
}

// --- State accessors -----------------------------------------------------------

// Returns true once the state line has gone HIGH (physical separation confirmed).
// Latched — returns true for the remainder of the session after separation.
bool QR_Slave::isSeparated() { return _separated; }

// Live state, no latching -- _prev_state is refreshed with the current GPIO
// read on every update() call, so this reflects reconnection immediately.
bool QR_Slave::isCurrentlyConnected() { return _prev_state == LOW; }

// --- Local override ----------------------------------------------------------

// Engage local release: servo opens on the next update() regardless of the release latch.
void QR_Slave::localRelease()
{
    _local_override = true;
}

// Release local override: servo follows the release-commanded latch on the next update().
void QR_Slave::localHold()
{
    _local_override = false;
}
