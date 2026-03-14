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
constexpr uint8_t AUX_IO_1_EIO =  8;   // Port B, pin B0  --  AUX connector 1, I/O 1
constexpr uint8_t AUX_IO_2_EIO =  9;   // Port B, pin B1  --  AUX connector 1, I/O 2
constexpr uint8_t AUX_IO_3_EIO = 10;   // Port B, pin B2  --  AUX connector 2, I/O 3
constexpr uint8_t AUX_IO_4_EIO = 11;   // Port B, pin B3  --  AUX connector 2, I/O 4
constexpr uint8_t AUX_IO_5_EIO = 12;   // Port B, pin B4  --  AUX connector 3, I/O 5
constexpr uint8_t AUX_IO_6_EIO = 13;   // Port B, pin B5  --  AUX connector 3, I/O 6

// --- AUX 1: Fill valve handshake (EMU ↔ GSEMU) --------------------------------
//
// I/O 1 — Fill valve command (EMU OUTPUT → GSEMU INPUT_PULLUP):
//   EMU drives this OUTPUT; GSEMU reads INPUT_PULLUP.
//   LOW  = EMU commands fill valve OPEN.
//   HIGH = EMU commands fill valve CLOSE / idle (safe default when disconnected).
constexpr uint8_t FILL_VALVE_CMD_EIO    = AUX_IO_1_EIO;

// I/O 2 — Fill valve status (GSEMU OUTPUT → EMU INPUT_PULLUP):
//   GSEMU drives this OUTPUT; EMU reads INPUT_PULLUP.
//   LOW  = fill valve is OPEN (filling).
//   HIGH = fill valve is CLOSED.
constexpr uint8_t FILL_VALVE_STATUS_EIO = AUX_IO_2_EIO;

// --- AUX 2: Umbilical quick-release (EMU ↔ GSEMU) ----------------------------
//
// I/O 3 — QR release command (EMU OUTPUT → GSEMU INPUT_PULLUP):
//   Level-based command.  GSEMU reads INPUT_PULLUP.
//   LOW  = EMU commands latch OPEN (release umbilical).
//   HIGH = EMU commands latch CLOSED / hold (safe default when disconnected).
constexpr uint8_t QR_CMD_EIO           = AUX_IO_3_EIO;

// I/O 4 — QR release state (GSEMU INPUT_PULLUP ← EMU GND connection):
//   GSEMU reads this as INPUT_PULLUP; it is wired to EMU chassis GND.
//   LOW  = umbilical connector is physically connected (EMU GND holds it LOW).
//   HIGH = umbilical connector has separated (INPUT_PULLUP floats HIGH).
constexpr uint8_t RELEASE_STATE_EIO    = AUX_IO_4_EIO;

// --- AUX 3: Remote launch enable and start (EMU ↔ GSEMU) ---------------------
//
// I/O 5 — Remote launch enable (EMU OUTPUT → GSEMU INPUT_PULLUP):
//   GSEMU reads INPUT_PULLUP.
//   LOW  = EMU has enabled remote engine start.
//   HIGH = Remote start disabled (safe default when disconnected).
constexpr uint8_t LAUNCH_ENABLE_EIO    = AUX_IO_5_EIO;

// I/O 6 — Remote start command (GSEMU OUTPUT → EMU INPUT_PULLUP):
//   GSEMU drives this OUTPUT; EMU reads INPUT_PULLUP.
//   LOW  = GSEMU commands engine start (asserted when AUX A/D is HIGH AND
//          LAUNCH_ENABLE_EIO is LOW).
//   HIGH = Hold / no start (safe default when disconnected).
constexpr uint8_t REMOTE_START_EIO     = AUX_IO_6_EIO;

// --- QR servo hardware configuration -----------------------------------------
// Calibrated March 2026 on the bench.
constexpr uint8_t  QR_SERVO_PWM_CHANNEL = 0;       // PCA9685 channel 0
constexpr int      QR_SERVO_PWM_HOLD    = 2400;    // 12-bit PWM count, latch engaged (not released)
constexpr int      QR_SERVO_PWM_OPEN    = 2800;    // 12-bit PWM count, latch released
constexpr uint16_t QR_SERVO_MOVE_MS     = 1000;    // ms — servo stroke time

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
