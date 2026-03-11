#ifndef KJO_QR_SLAVE_H
#define KJO_QR_SLAVE_H

//
//    Ken Overton
//    QR_Slave  --  GSEMU-side umbilical quick-release actuator
//
//    Controls the spring-loaded latch servo on the GSEMU in response to a
//    level-based command from the EMU over AUX connector 2.
//
//      CMD line (I/O 3, AUX_IO_3 / QR_CMD_EIO):
//        GSEMU reads this as INPUT_PULLUP.
//        LOW  = EMU commands latch OPEN  (release umbilical).
//        HIGH = EMU commands latch CLOSED / hold.
//        When umbilical disconnects, INPUT_PULLUP floats HIGH → latch holds (safe default).
//
//      State line (I/O 4, AUX_IO_4 / RELEASE_STATE_EIO):
//        GSEMU reads this as INPUT_PULLUP.
//        It is wired to EMU chassis GND via the AUX connector.
//        LOW  = umbilical connector is physically connected (EMU GND pulls it LOW).
//        HIGH = umbilical connector has separated (INPUT_PULLUP floats HIGH).
//
//    Servo behaviour:
//      update() polls the CMD level every iteration.
//      - CMD LOW  and servo not already opening/open  → openServo()
//      - CMD HIGH and servo not already holding/held  → holdServo()
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
        //   plus GPIO expander pointer, command pin, and state pin.
        QR_Slave( byte PWM_channel, int PWM_hold, int PWM_open,
                  uint16_t move_time_ms, Adafruit_PWMServoDriver *Servos,
                  Adafruit_MCP23X17 *gpio, byte cmd_pin, byte state_pin );

        // Call once from setup() — after the servo wing has been initialised.
        //   - Configures cmd_pin   as INPUT_PULLUP (receives level command from EMU).
        //   - Configures state_pin as INPUT_PULLUP (reads physical separation via EMU GND).
        //   - Commands servo to HOLD (latch engaged).
        void begin();

        // Call every loop() iteration — non-blocking.
        // Follows the CMD level and drives the servo accordingly.
        // Detects physical separation on the state line and latches the event.
        void update();

        // Returns true once the state line has gone HIGH (physical separation confirmed).
        // Latched — stays true after the first HIGH transition.
        bool isSeparated();

        // Returns true while the umbilical is physically connected (state line LOW).
        bool isConnected();

        // Local override — bypasses the CMD line and forces the servo to open.
        // Intended for front-panel Button A (hold-to-release):
        //   Call localRelease() on button press   (Button A goes LOW).
        //   Call localHold()    on button release  (Button A returns HIGH).
        // While the override is active, the CMD line is ignored.
        void localRelease();    // set override; servo opens on next update()
        void localHold();       // clear override; servo follows CMD line on next update()

        // Returns true while the local override is active.
        bool isLocalOverride();

    private:
        Adafruit_MCP23X17 *_gpio;
        byte               _cmd_pin;
        byte               _state_pin;
        bool               _prev_state;      // previous sample of state_pin
        bool               _separated;       // latched separation flag
        bool               _local_override;  // true = Button A held, ignore CMD line
};

#endif // KJO_QR_SLAVE_H
