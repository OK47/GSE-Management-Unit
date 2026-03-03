#ifndef KJO_SLAVE_VALVE_H
#define KJO_SLAVE_VALVE_H

//
//    Ken Overton
//    Slave_Valve  --  GSEMU-side servo valve driven by EMU GPIO handshake
//
//    The Fill valve is physically located on the GSE and actuated by the GSEMU.
//    The GSEMU monitors a command bit driven by the EMU, operates the valve servo
//    in response, and reports valve position back to the EMU via a status bit.
//
//      Command bit (FILL_VALVE_CMD_EIO):
//        EMU drives this as OUTPUT; GSEMU reads it as INPUT_PULLUP.
//        Falling edge (HIGH→LOW) = EMU commands valve open.
//        Rising  edge (LOW→HIGH) = EMU commands valve closed.
//
//      Status bit (FILL_VALVE_STATUS_EIO):
//        GSEMU drives this as OUTPUT.
//        LOW  = valve is open.
//        HIGH = valve is closed (idle/default state).
//        The status bit is updated after each move completes (isStopped() == true).
//        There is no "moving" state on this bit — it retains its previous value
//        during valve travel.
//
//    Slave_Valve inherits from Valve and reuses all servo, encoder, position,
//    and motion-detection machinery.  The additional update() method must be
//    called every loop() iteration; it watches for command-bit edges and
//    dispatches the corresponding open()/close() servo move.
//
//    open() and close() are inherited from Valve and remain usable directly,
//    but in normal operation they are called exclusively by update().
//    openTo() is inherited but should not be used — Slave_Valve only supports
//    fully open or fully closed operation.
//

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "KJO_Valve.h"          // Valve base class

class Slave_Valve : public Valve
{
    public:
        // Constructor: all Valve arguments plus GPIO expander, command pin, status pin.
        Slave_Valve( byte use,
                     byte PWM_channel,    int PWM_open,    int PWM_close,
                     byte encoder_channel, int encoder_open, int encoder_close,
                     float ball_OD, float bore_OD, float throat_ID,
                     Adafruit_PWMServoDriver *Servos,
                     Adafruit_MCP23X17 *gpio, byte cmd_pin, byte status_pin );

        // Call once from setup() — after servo wing init.
        // Calls Valve::begin() (closes valve to known state), then:
        //   - configures cmd_pin as INPUT_PULLUP
        //   - configures status_pin as OUTPUT, drives HIGH (= closed)
        void begin();

        // Call every loop() iteration.
        // Detects falling / rising edges on cmd_pin and dispatches open()/close().
        // Updates status_pin after each move completes.
        void update();

    private:
        Adafruit_MCP23X17 *_gpio;
        byte _cmd_pin;
        byte _status_pin;
        bool _prev_cmd;         // previous sample of cmd_pin (for edge detection)
        bool _move_pending;     // true while a servo move is in progress
        bool _target_open;      // true = moving to open, false = moving to closed
};

#endif // KJO_SLAVE_VALVE_H
