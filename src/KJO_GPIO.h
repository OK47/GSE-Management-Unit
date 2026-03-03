#ifndef KJO_GPIO_H
#define KJO_GPIO_H

//
//    Ken Overton
//    GPIO subsystem  --  expansion I/O and relay driver
//
//    Pin assignments for the Adafruit MCP23X17 16-bit GPIO expander (I2C),
//    and the Relay class that wraps an expander output pin to drive
//    buzzer and relay loads with optional timed operation.
//

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

// --- MCP23X17 expansion GPIO pin assignments ----------------------------------
// Port A pins (0-7)
constexpr uint8_t BUTTON_A_EIO =  3;   // Port A, pin A3  --  display button A
constexpr uint8_t BUTTON_B_EIO =  4;   // Port A, pin A4  --  display button B
constexpr uint8_t BUTTON_C_EIO =  5;   // Port A, pin A5  --  display button C
constexpr uint8_t BUZZER_EIO   =  6;   // Port A, pin A6  --  audible alarm buzzer relay

// Port B pins (8-15)
constexpr uint8_t AUX_IO_1_EIO =  8;   // Port B, pin B0  --  auxiliary output 1
constexpr uint8_t AUX_IO_2_EIO =  9;   // Port B, pin B1  --  auxiliary output 2
constexpr uint8_t AUX_IO_3_EIO = 10;   // Port B, pin B2  --  auxiliary output 3
constexpr uint8_t AUX_IO_4_EIO = 11;   // Port B, pin B3  --  auxiliary output 4

// --- EMU handshake pin aliases ------------------------------------------------
// AUX_IO_1 and AUX_IO_2 are wired to matching pins on the EMU via the AUX
// connector.  Note: directions are the REVERSE of the EMU-side definitions.
//
// Fill valve command (EMU → GSEMU):
//   GSEMU reads this INPUT_PULLUP; LOW = open command, HIGH = close/idle.
constexpr uint8_t FILL_VALVE_CMD_EIO    = AUX_IO_1_EIO;

// Fill valve status (GSEMU → EMU):
//   GSEMU drives this OUTPUT; LOW = open, HIGH = closed.
constexpr uint8_t FILL_VALVE_STATUS_EIO = AUX_IO_2_EIO;

// AUX_IO_3 and AUX_IO_4 are reserved for the umbilical-release handshake (TBD).

// --- Relay class --------------------------------------------------------------
//
// Wraps a single MCP23X17 output pin to drive a relay (or any digital load).
// Supports momentary (timed) or continuous on/off operation.
// Call update() regularly from the main loop to support timed-off operation.
//
class Relay
{
    public:
        Relay( Adafruit_MCP23X17 *E_GPIO_ptr, short pin );
        void begin();

        // Continuous control
        void close();                   // Energise the relay (close contacts)
        void on();                      // Alias for close()
        void open();                    // De-energise the relay (open contacts)
        void off();                     // Alias for open()

        // Timed control  --  relay opens automatically after 'duration' ms
        void close( long duration );
        void on( long duration );       // Alias for close(duration)

        // Configuration
        void  setControlPin( short pin );
        short getControlPin();

        // State queries
        bool  isOn();
        long  getOnTime();

        // Must be called from loop() to support timed-off operation
        void update();

    private:
        bool                _on_timer    = false;
        short               _control_pin = -1;
        long                _open_time   = 0;
        Adafruit_MCP23X17  *_E_GPIO_ptr;
};

#endif // KJO_GPIO_H
