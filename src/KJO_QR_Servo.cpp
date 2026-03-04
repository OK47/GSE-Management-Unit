#include "KJO_QR_Servo.h"

//
// QR_Servo — servo-only actuator with time-based motion detection.
// The PCA9685 servo driver must be initialised by the caller before begin().
//

QR_Servo::QR_Servo( byte PWM_channel, int PWM_hold, int PWM_open,
                    uint16_t move_time_ms, Adafruit_PWMServoDriver *Servos )
    : _channel( PWM_channel ), _PWM_hold( PWM_hold ), _PWM_open( PWM_open ),
      _move_time_ms( move_time_ms ), _move_start_ms( 0 ),
      _servo_state( QR_Servo_State::AT_HOLD ), _Servos( Servos )
{
}

// ---------------------------------------------------------------------------
void QR_Servo::begin()
{
    // The servo wing was already initialised by main.cpp before begin() is
    // called.  Just command the hold (latch-engaged) position.
    holdServo();
}

// ---------------------------------------------------------------------------
void QR_Servo::openServo()
{
    _Servos->setPWM( _channel, 0, _PWM_open );
    _move_start_ms = millis();
    _servo_state   = QR_Servo_State::MOVING_TO_OPEN;
}

void QR_Servo::holdServo()
{
    _Servos->setPWM( _channel, 0, _PWM_hold );
    _move_start_ms = millis();
    _servo_state   = QR_Servo_State::MOVING_TO_HOLD;
}

// ---------------------------------------------------------------------------
// Returns true while the move window is still open; updates state on expiry.
bool QR_Servo::isMoving()
{
    if( _servo_state == QR_Servo_State::AT_HOLD ||
        _servo_state == QR_Servo_State::AT_OPEN )
    {
        return false;
    }

    if( millis() - _move_start_ms >= _move_time_ms )
    {
        _servo_state = ( _servo_state == QR_Servo_State::MOVING_TO_OPEN )
                       ? QR_Servo_State::AT_OPEN
                       : QR_Servo_State::AT_HOLD;
        return false;
    }
    return true;
}

bool QR_Servo::isAtOpen()
{
    isMoving();   // update state if timer has expired
    return _servo_state == QR_Servo_State::AT_OPEN;
}

bool QR_Servo::isAtHold()
{
    isMoving();   // update state if timer has expired
    return _servo_state == QR_Servo_State::AT_HOLD;
}

QR_Servo_State QR_Servo::getServoState() { return _servo_state; }
