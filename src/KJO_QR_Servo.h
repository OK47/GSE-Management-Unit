#ifndef KJO_QR_SERVO_H
#define KJO_QR_SERVO_H

//
//    Ken Overton
//    QR_Servo  --  Servo-only (no position feedback) actuator base class
//
//    Used for the umbilical quick-release latch servo on the GSEMU.
//    Position is tracked by commanded state rather than a potentiometer;
//    motion completion is determined by elapsed time.
//
//    Positions:
//      HOLD = servo drives latch to engaged position
//      OPEN = servo drives latch to released position
//
//    Call openServo() / holdServo() to start a move (non-blocking).
//    isMoving(), isAtOpen(), and isAtHold() query state.
//

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Internal position state
enum class QR_Servo_State
{
    AT_HOLD,
    MOVING_TO_OPEN,
    AT_OPEN,
    MOVING_TO_HOLD
};

class QR_Servo
{
    public:
        // Constructor.
        //   PWM_channel  : PCA9685 output channel (0-based)
        //   PWM_hold     : 12-bit PWM count for the latch-engaged position
        //   PWM_open     : 12-bit PWM count for the latch-released position
        //   move_time_ms : time allowed for a complete servo stroke (ms)
        //   Servos       : pointer to the shared PWM servo driver
        QR_Servo( byte PWM_channel, int PWM_hold, int PWM_open,
                  uint16_t move_time_ms, Adafruit_PWMServoDriver *Servos );

        // Call once from setup() (after the servo driver has been initialised
        // by the caller).  Commands servo to HOLD position.
        void begin();

        // Command servo to the latch-released (open) position. Non-blocking.
        void openServo();

        // Command servo to the latch-engaged (hold) position. Non-blocking.
        void holdServo();

        // Returns true while the timed move window is still open.
        // Also updates the internal state to AT_OPEN or AT_HOLD when the
        // window expires — safe to call every loop iteration.
        bool isMoving();

        // Returns true when servo has been commanded open and move time has elapsed.
        bool isAtOpen();

        // Returns true when servo has been commanded hold and move time has elapsed.
        bool isAtHold();

        QR_Servo_State getServoState();

    protected:
        byte                     _channel;
        int                      _PWM_hold;
        int                      _PWM_open;
        uint16_t                 _move_time_ms;
        unsigned long            _move_start_ms;
        QR_Servo_State           _servo_state;
        Adafruit_PWMServoDriver *_Servos;
};

#endif // KJO_QR_SERVO_H
