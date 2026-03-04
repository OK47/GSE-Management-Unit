#include "KJO_QR_Slave.h"
#include "KJO_GPIO.h"   // QR_RELEASE_HOLD_MS

//
// QR_Slave — GSEMU-side umbilical quick-release actuator.
// See KJO_QR_Slave.h for full protocol description.
//

QR_Slave::QR_Slave( byte PWM_channel, int PWM_hold, int PWM_open,
                    uint16_t move_time_ms, Adafruit_PWMServoDriver *Servos,
                    Adafruit_MCP23X17 *gpio, byte cmd_pin )
    : QR_Servo( PWM_channel, PWM_hold, PWM_open, move_time_ms, Servos ),
      _gpio( gpio ), _cmd_pin( cmd_pin ),
      _prev_cmd( HIGH ), _edge_count( 0 ),
      _release_time_ms( 0 ), _state( QR_State::ARMED )
{
}

// ---------------------------------------------------------------------------
void QR_Slave::begin()
{
    // Configure the CMD line as INPUT_PULLUP.  While the umbilical is
    // connected the EMU drives it LOW, overriding the pullup.
    _gpio->pinMode( _cmd_pin, INPUT_PULLUP );
    _prev_cmd = _gpio->digitalRead( _cmd_pin );
    _edge_count = 0;
    _state = QR_State::ARMED;

    // Command servo to hold (latch engaged).
    QR_Servo::begin();
}

// ---------------------------------------------------------------------------
// Non-blocking state machine.  Must be called every loop() iteration.
void QR_Slave::update()
{
    bool curr_cmd    = _gpio->digitalRead( _cmd_pin );
    bool rising_edge = ( curr_cmd == HIGH && _prev_cmd == LOW );
    _prev_cmd = curr_cmd;

    switch( _state )
    {
        case QR_State::ARMED:
            if( rising_edge )
            {
                // Edge #1: EMU release-intent pulse (HIGH for ~QR_RELEASE_PULSE_MS).
                _edge_count = 1;
                openServo();
                _state = QR_State::OPENING;
            }
            break;

        case QR_State::OPENING:
            // Continue watching for Edge #2 while servo is moving.
            // Edge #2 can arrive before the servo reaches open position —
            // handle it immediately regardless of servo completion.
            if( rising_edge )
            {
                // Edge #2: physical connector separation, INPUT_PULLUP wins.
                _edge_count      = 2;
                _release_time_ms = millis();
                _state           = QR_State::RELEASED;
            }
            break;

        case QR_State::RELEASED:
            // Dwell at open position to ensure latch has fully cleared,
            // then retract the servo.
            if( millis() - _release_time_ms >= QR_RELEASE_HOLD_MS )
            {
                holdServo();
                _state = QR_State::CLOSING;
            }
            break;

        case QR_State::CLOSING:
            // Wait for timed servo retraction to complete.
            if( !isMoving() )
                _state = QR_State::DONE;
            break;

        case QR_State::DONE:
            // Terminal state — no further action.
            break;
    }
}

// ---------------------------------------------------------------------------
QR_State QR_Slave::getState()  { return _state; }
bool     QR_Slave::isReleased(){ return ( _state == QR_State::RELEASED ||
                                          _state == QR_State::CLOSING   ||
                                          _state == QR_State::DONE ); }
bool     QR_Slave::isDone()    { return _state == QR_State::DONE; }
