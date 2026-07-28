#ifndef KJO_QR_SLAVE_H
#define KJO_QR_SLAVE_H

//
//    Ken Overton
//    QR_Slave  --  GSEMU-side umbilical quick-release actuator
//
//    Controls the spring-loaded latch servo on the GSEMU. Release is
//    commanded over CAN (CAN_QR_RELEASE, sent fire-and-forget from EMU
//    via Send_CAN_Command_NoWait() -- see Check_CAN()'s dispatch in
//    main.cpp) instead of a dedicated wire. Release is one-shot: once
//    commandRelease() is called, the servo opens and stays open -- there
//    is no remote command back to "hold." A spring-loaded latch, once
//    triggered, cannot be electrically un-released; only physical
//    re-latching (and/or the local front-panel override below) changes
//    it. See docs/superpowers/specs/2026-07-22-qr-can-release-design.md
//    (and its 2026-07-28 correction note).
//
//      State line (QR_SENSE_EIO, AUX connector 2, I/O 3):
//        GSEMU reads this as INPUT_PULLUP.
//        It is wired to EMU chassis GND via the AUX connector.
//        LOW  = umbilical connector is physically connected (EMU GND pulls it LOW).
//        HIGH = umbilical connector has separated (INPUT_PULLUP floats HIGH).
//
//    Servo behaviour:
//      update() polls every iteration.
//      - local override active OR release commanded, and servo not
//        already opening/open  → openServo()
//      - otherwise, and servo not already holding/held  → holdServo()
//      Physical separation (state line LOW → HIGH) is detected and latched;
//      isSeparated() returns true once separation has been confirmed.
//
//    Call begin() once from setup() (after the servo wing is initialised).
//    Call update() every loop() iteration — non-blocking.
//

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "KJO_QR_Servo.h"

class QR_Slave : public QR_Servo
{
    public:
        // Constructor.
        //   All QR_Servo arguments (PWM channel, hold/open PWM, move time, servo driver)
        //   plus GPIO expander pointer and state pin.
        QR_Slave( byte PWM_channel, int PWM_hold, int PWM_open,
                  uint16_t move_time_ms, Adafruit_PWMServoDriver *Servos,
                  Adafruit_MCP23X17 *gpio, byte state_pin );

        // Call once from setup() — after the servo wing has been initialised.
        //   - Configures state_pin as INPUT_PULLUP (reads physical separation via EMU GND).
        //   - Commands servo to HOLD (latch engaged).
        void begin();

        // Call every loop() iteration — non-blocking.
        // Opens the servo once local override or a release command is
        // active; detects physical separation on the state line and
        // latches the event.
        void update();

        // Latches the release command (permanent -- never cleared). Call
        // from Check_CAN()'s CAN_QR_RELEASE case. The servo opens on the
        // next update() and stays open.
        void commandRelease();

        // Returns true once the state line has gone HIGH (physical separation confirmed).
        // Latched — stays true after the first HIGH transition.
        bool isSeparated();

        // Returns true while the umbilical is physically connected (state line LOW).
        bool isConnected();

        // Local override — bypasses the release-commanded latch and forces the servo to open.
        // Intended for front-panel Button A (hold-to-release):
        //   Call localRelease() on button press   (Button A goes LOW).
        //   Call localHold()    on button release  (Button A returns HIGH).
        void localRelease();    // set override; servo opens on next update()
        void localHold();       // clear override; servo follows the release-commanded latch on next update()

        // Returns true while the local override is active.
        bool isLocalOverride();

    private:
        Adafruit_MCP23X17 *_gpio;
        byte               _state_pin;
        bool               _prev_state;         // previous sample of state_pin
        bool               _separated;          // latched separation flag
        bool               _local_override;      // true = Button A held, forces open
        bool               _release_commanded;   // true = CAN_QR_RELEASE received; permanent latch
};

#endif // KJO_QR_SLAVE_H
