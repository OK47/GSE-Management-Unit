#ifndef KJO_QR_SLAVE_H
#define KJO_QR_SLAVE_H

//
//    Ken Overton
//    QR_Slave  --  GSEMU-side umbilical quick-release actuator
//
//    The umbilical connector is held by a spring-loaded latch driven by this
//    servo.  The CMD line (AUX_IO_3) is shared with the EMU:
//
//      CMD (EMU → GSEMU):
//        GSEMU reads this as INPUT_PULLUP.
//        While the umbilical is connected the EMU drives it LOW; when the
//        connector separates the EMU's drive disappears and INPUT_PULLUP wins,
//        taking the line HIGH naturally.
//
//    Release sequence (rising-edge detection):
//      Edge #1  (EMU pulses HIGH for ~QR_RELEASE_PULSE_MS):
//               EMU signals release intent → servo starts opening.
//      Edge #2  (connector separates, INPUT_PULLUP wins):
//               Physical separation confirmed → start QR_RELEASE_HOLD_MS dwell.
//      After dwell: servo retracts to HOLD.
//
//    State machine:
//      ARMED → OPENING → RELEASED → CLOSING → DONE
//
//    Call update() every loop() iteration; it is non-blocking.
//

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "KJO_QR_Servo.h"

enum class QR_State
{
    ARMED,     // Normal armed state: latch engaged, watching for Edge #1
    OPENING,   // Servo driving to open position; watching for Edge #2
    RELEASED,  // Connector confirmed separated; waiting QR_RELEASE_HOLD_MS
    CLOSING,   // Servo retracting to hold
    DONE       // Terminal state: servo retracted, umbilical gone
};

class QR_Slave : public QR_Servo
{
    public:
        // Constructor.  All QR_Servo arguments plus GPIO expander and command pin.
        QR_Slave( byte PWM_channel, int PWM_hold, int PWM_open,
                  uint16_t move_time_ms, Adafruit_PWMServoDriver *Servos,
                  Adafruit_MCP23X17 *gpio, byte cmd_pin );

        // Call once from setup() — after the servo wing has been initialised.
        //   - Calls QR_Servo::begin() (commands servo to HOLD).
        //   - Configures cmd_pin as INPUT_PULLUP.
        void begin();

        // Call every loop() iteration — non-blocking.
        // Detects rising edges on cmd_pin and advances the state machine.
        void update();

        // State accessors
        QR_State getState();
        bool isReleased();   // true once connector has separated (RELEASED / CLOSING / DONE)
        bool isDone();       // true when servo has retracted post-release (DONE)

    private:
        Adafruit_MCP23X17 *_gpio;
        byte               _cmd_pin;
        bool               _prev_cmd;
        uint8_t            _edge_count;
        unsigned long      _release_time_ms;   // millis() timestamp of Edge #2
        QR_State           _state;
};

#endif // KJO_QR_SLAVE_H
