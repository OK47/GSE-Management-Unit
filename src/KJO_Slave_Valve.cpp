#include "KJO_Slave_Valve.h"

//
// Slave_Valve method implementations.
//

// --- Constructor -------------------------------------------------------------
//
// Passes all Valve arguments through to the Valve base-class constructor, then
// stores the additional GPIO expander pointer and handshake pin numbers.
//
Slave_Valve::Slave_Valve( byte use,
                           byte PWM_channel,    int PWM_open,    int PWM_close,
                           byte encoder_channel, int encoder_open, int encoder_close,
                           float ball_OD, float bore_OD, float throat_ID,
                           Adafruit_PWMServoDriver *Servos,
                           Adafruit_MCP23X17 *gpio, byte cmd_pin, byte status_pin )
    : Valve( use,
             PWM_channel,    PWM_open,    PWM_close,
             encoder_channel, encoder_open, encoder_close,
             ball_OD, bore_OD, throat_ID,
             Servos )
{
    _gpio         = gpio;
    _cmd_pin      = cmd_pin;
    _status_pin   = status_pin;
    _prev_cmd     = HIGH;    // pull-up default = HIGH (closed command)
    _move_pending = false;
    _target_open  = false;
}

// --- begin() -----------------------------------------------------------------
//
// 1. Calls Valve::begin(), which computes geometry, then closes the valve to a
//    known state and waits for isStopped().
// 2. Configures the command pin as INPUT_PULLUP (driven by the EMU).
// 3. Configures the status pin as OUTPUT and drives it HIGH to report "closed".
//
void Slave_Valve::begin()
{
    Valve::begin();    // close valve to known state

    _gpio->pinMode(      _cmd_pin,    INPUT_PULLUP );
    _gpio->pinMode(      _status_pin, OUTPUT       );
    _gpio->digitalWrite( _status_pin, HIGH         );  // report: valve is closed

    // Initialise edge-detection state to match the quiescent (pulled-up) level.
    _prev_cmd = HIGH;
}

// --- update() ----------------------------------------------------------------
//
// Must be called from loop() on every iteration.
//
// Edge detection on the command pin:
//   Falling edge (HIGH → LOW): EMU commands valve OPEN  → call Valve::open()
//   Rising  edge (LOW → HIGH): EMU commands valve CLOSE → call Valve::close()
//
// Status pin management:
//   While a move is pending and isStopped() becomes true, update the status pin
//   to reflect the new valve position and clear the pending flag.
//
void Slave_Valve::update()
{
    bool curr_cmd = _gpio->digitalRead( _cmd_pin );

    // ── Falling edge: open command ───────────────────────────────────────────
    if( curr_cmd == LOW && _prev_cmd == HIGH )
    {
        Valve::open();          // start non-blocking servo move
        _move_pending = true;
        _target_open  = true;
    }

    // ── Rising edge: close command ───────────────────────────────────────────
    if( curr_cmd == HIGH && _prev_cmd == LOW )
    {
        Valve::close();         // start non-blocking servo move
        _move_pending = true;
        _target_open  = false;
    }

    _prev_cmd = curr_cmd;       // save for next iteration

    // ── Update status pin when move completes ────────────────────────────────
    // isStopped() is checked unconditionally so the status pin is updated even
    // if the valve finishes before the next command-edge arrives.
    if( _move_pending && isStopped() )
    {
        // Drive status LOW if valve reached open, HIGH if closed (or failed).
        _gpio->digitalWrite( _status_pin, isOpen() ? LOW : HIGH );
        _move_pending = false;
    }
}
