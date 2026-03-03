/***************************************************
  GSE Management Unit (GSEMU)

  The GSEMU resides on the ground and controls all
  Ground Support Equipment (GSE) valves and functions.
  It communicates with the Engine Management Unit (EMU)
  over a small number of digital lines connected to
  the Expansion I/O board.

  Hardware:
    - Adafruit Feather M0
    - Adafruit SH1107 128x64 OLED display (I2C)
    - Adafruit MCP23X17 GPIO expander (I2C)   [buttons + buzzer + aux I/O]
    - Adafruit PWM Servo Wing (I2C)
    - PCF8523 Real-time clock (I2C)
    - SD card (SPI)

  Ken Overton  (C) 2025
****************************************************/

// --- Arduino / library includes ----------------------------------------------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MCP23X17.h>

// --- Subsystem headers --------------------------------------------------------
#include <KJO_System_Config.h>      // Versions, platform, USE_MAPPING
#include "KJO_Logging.h"            // SD card CS, log file naming
#include "KJO_GPIO.h"               // GPIO expander pins + Relay class
#include "KJO_Valve.h"              // Valve base class + servo/motion constants
#include "KJO_GSE_Display.h"        // OLED scrollMessage() + Find_Available_File()
#include "KJO_Slave_Valve.h"        // GSEMU-side servo valve driven by EMU GPIO handshake

// --- Conditional serial console ----------------------------------------------
// Uncomment to enable Serial output for debugging.
#define SERIAL_CONSOLE_OUTPUT

// --- Platform identifier for this build --------------------------------------
constexpr uint8_t PLATFORM = GSEMU_PLATFORM;

// --- Hardware object instantiation -------------------------------------------

// Real-time clock (I2C)
RTC_PCF8523 RT_Clock;

// OLED display (I2C, 64 x 128 px)
Adafruit_SH1107 Screen = Adafruit_SH1107( 64, 128, &Wire );

// MCP23X17 GPIO expander (I2C)
Adafruit_MCP23X17 E_GPIO;

// Buzzer relay (uses the GPIO expander)
Relay Buzzer = Relay( &E_GPIO, BUZZER_EIO );

// PWM servo driver (I2C, Adafruit ServoWing)
Adafruit_PWMServoDriver Servos = Adafruit_PWMServoDriver();

// Fill valve  --  servo-actuated, driven by EMU GPIO handshake via Slave_Valve.
// Hardware constants from KJO_Valve.h; GPIO pins from KJO_GPIO.h.
Slave_Valve Fill_Valve( FILL_VALVE,
                        FILL_VALVE_PWM, FILL_VALVE_PWM_OPEN, FILL_VALVE_PWM_CLOSE,
                        FILL_VALVE_ANALOG_PIN, FILL_VALVE_POS_OPEN, FILL_VALVE_POS_CLOSED,
                        FILL_VALVE_BALL_DIAMETER, FILL_VALVE_BORE_DIAMETER, FILL_VALVE_THROAT_DIAMETER,
                        &Servos,
                        &E_GPIO, FILL_VALVE_CMD_EIO, FILL_VALVE_STATUS_EIO );

// SD card log file
SdFile Log_file;
String log_file_name;

// --- Forward declarations -----------------------------------------------------
void Test_Buzzer();
void Beep( short count );
void Check_Buttons();
void Post_Log_Message( String S );

// -----------------------------------------------------------------------------
void setup()
{
#ifdef SERIAL_CONSOLE_OUTPUT
    Serial.begin( 115200 );
    // Wait up to 3 s for USB-CDC enumeration (avoids blocking when untethered)
    { unsigned long _t = millis() + 3000; while( !Serial && millis() < _t ) delay( 10 ); }
    Serial.println( "Starting up." );
#endif

    // -- OLED display ----------------------------------------------------------
    Screen.begin();
    Screen.clearDisplay();
    Screen.display();
    Screen.setRotation( 3 );
    Screen.setTextColor( SH110X_WHITE );
    Screen.setTextSize( 2 );
    Screen.setTextWrap( false );
    delay( 2000 );
    Screen.clearDisplay();
    Screen.display();

#ifdef SERIAL_CONSOLE_OUTPUT
    Serial.println( "GSEMU starting." );
#endif
    scrollMessage( &Screen, "GSEMU starting.", true );

#ifdef SERIAL_CONSOLE_OUTPUT
    Serial.println( "Display ready." );
#endif
    scrollMessage( &Screen, "Display ready.", true );

    // -- GPIO expansion board --------------------------------------------------
    if( !E_GPIO.begin_I2C() )
    {
#ifdef SERIAL_CONSOLE_OUTPUT
        Serial.println( "Expansion I/O Error." );
#endif
        scrollMessage( &Screen, "Expansion I/O ERROR.", true );
    }
    else
    {
#ifdef SERIAL_CONSOLE_OUTPUT
        Serial.println( "Expansion I/O ready." );
#endif
        scrollMessage( &Screen, "Expansion I/O ready.", true );
    }

    // Configure expansion I/O pins
    E_GPIO.pinMode(    BUTTON_A_EIO, INPUT_PULLUP );
    E_GPIO.pinMode(    BUTTON_B_EIO, INPUT_PULLUP );
    E_GPIO.pinMode(    BUTTON_C_EIO, INPUT_PULLUP );
    // AUX_IO_1 (FILL_VALVE_CMD_EIO) and AUX_IO_2 (FILL_VALVE_STATUS_EIO) are
    // configured by Slave_Valve::begin() called below with Fill_Valve.begin().
    // AUX_IO_3 and AUX_IO_4 are reserved for umbilical-release handshake (TBD).
    E_GPIO.pinMode(    AUX_IO_3_EIO, OUTPUT );  E_GPIO.digitalWrite( AUX_IO_3_EIO, LOW );
    E_GPIO.pinMode(    AUX_IO_4_EIO, OUTPUT );  E_GPIO.digitalWrite( AUX_IO_4_EIO, LOW );

    // -- SD card ---------------------------------------------------------------
    if( !SD.begin( CARD_CS ) )
    {
        scrollMessage( &Screen, "SD card Failed.", true );
    }
    else
    {
        scrollMessage( &Screen, "SD card ready.", true );
        log_file_name = Find_Available_File();
        scrollMessage( &Screen, log_file_name, true );
    }

    // -- Real-time clock -------------------------------------------------------
    if( !RT_Clock.begin() )
    {
        scrollMessage( &Screen, "RTC fail.", true );
    }
    else if( RT_Clock.lostPower() )
    {
        // Set clock to sketch compile time if battery was lost
        RT_Clock.adjust( DateTime( F(__DATE__), F(__TIME__) ) );
        scrollMessage( &Screen, "RTC time set.", true );
    }
    else
    {
        scrollMessage( &Screen, "RTC ready.", true );
    }

    // -- Buzzer ----------------------------------------------------------------
    Buzzer.begin();
    scrollMessage( &Screen, "Buzzer ready.", true );
    Test_Buzzer();

    // -- Servo wing ------------------------------------------------------------
    Servos.begin();
    Servos.setPWMFreq( SERVO_FREQUENCY );
    scrollMessage( &Screen, "Servos ready.", true );

    // -- Onboard ADC resolution ------------------------------------------------
    analogReadResolution( 12 );
    scrollMessage( &Screen, "Analog 12 bits.", true );

    // -- Fill valve (Slave_Valve: closes to known state, configures GPIO pins) --
    Fill_Valve.begin();
    scrollMessage( &Screen, "Fill valve ready.", true );

    scrollMessage( &Screen, "GSEMU ready.", true );
    Post_Log_Message( "GSEMU startup complete, version " + String( GSEMU_VERSION ) );
}

// -----------------------------------------------------------------------------
void loop()
{
    // Service timed relay outputs
    Buzzer.update();

    // Service Fill valve: check command bit and update servo + status bit
    Fill_Valve.update();

    // Check front-panel buttons (local control)
    Check_Buttons();
}

// -----------------------------------------------------------------------------
// Self-test: three short buzzer beeps.
// NOTE: Uses explicit on()/off() with blocking delays rather than the non-blocking
// on(duration) form, because this function is called from setup() where loop()
// (and therefore Relay::update()) never runs to service the timer.
void Test_Buzzer()
{
    for( short i = 0; i < 3; i++ )
    {
        Buzzer.on();    // energise immediately
        delay( 75 );   // hold for 75 ms
        Buzzer.off();   // de-energise immediately
        delay( 75 );   // silent gap between beeps
    }
}

// -----------------------------------------------------------------------------
// Audible confirmation: 'count' short blocking beeps (100 ms on, 100 ms gap).
//
// Uses delay() and is therefore BLOCKING.  This is acceptable anywhere in this
// application because scrollMessage() is also blocking, so loop() is already
// interrupted during button-response sequences.  Beeps play before the scroll.
void Beep( short count )
{
    for( short i = 0; i < count; i++ )
    {
        Buzzer.on();   delay( 100 );   Buzzer.off();
        if( i < count - 1 ) delay( 100 );   // gap between beeps
    }
}

// -----------------------------------------------------------------------------
// Poll the three front-panel buttons and act on falling-edge (press) events.
// Buttons are active-LOW (INPUT_PULLUP on MCP23X17 expansion GPIO).
//
// Button handlers are stubs — functionality will be defined once the GSE
// valve class and control sequences are established.
//
void Check_Buttons()
{
    static bool prev_A = HIGH;
    static bool prev_B = HIGH;
    static bool prev_C = HIGH;

    bool curr_A = E_GPIO.digitalRead( BUTTON_A_EIO );
    bool curr_B = E_GPIO.digitalRead( BUTTON_B_EIO );
    bool curr_C = E_GPIO.digitalRead( BUTTON_C_EIO );

    // Button A  --  display Fill valve position status (3 beeps)
    if( curr_A == LOW && prev_A == HIGH )
    {
        Beep( 3 );
        scrollMessage( &Screen,
                       Fill_Valve.isOpen()   ? "Fill: OPEN"   :
                       Fill_Valve.isClosed() ? "Fill: CLOSED" : "Fill: MOVING",
                       true );
    }

    // Button B  --  manually command Fill valve CLOSED (2 beeps)
    // NOTE: In normal operation the EMU drives open/close via the command bit.
    //       These buttons allow local testing of the valve without the EMU.
    if( curr_B == LOW && prev_B == HIGH )
    {
        scrollMessage( &Screen, "Fill CLOSED", true );
        Beep( 2 );
        Fill_Valve.close();
    }

    // Button C  --  manually command Fill valve OPEN (1 beep)
    if( curr_C == LOW && prev_C == HIGH )
    {
        scrollMessage( &Screen, "Fill OPEN", true );
        Beep( 1 );
        Fill_Valve.open();
    }

    prev_A = curr_A;
    prev_B = curr_B;
    prev_C = curr_C;
}

// -----------------------------------------------------------------------------
// Appends a timestamped message to the SD card log file.
// Also echoes to Serial when SERIAL_CONSOLE_OUTPUT is defined.
void Post_Log_Message( String S )
{
    DateTime now = RT_Clock.now();
    int hour = now.hour();
    if( hour > 12 ) hour -= 12;

    char h_string[4], m_string[4], s_string[4];
    sprintf( h_string, "%2d",  hour          );
    sprintf( m_string, "%02d", now.minute()  );
    sprintf( s_string, "%02d", now.second()  );

    String message = String(h_string) + ":" + String(m_string) + ":" + String(s_string) + " " + S;

#ifdef SERIAL_CONSOLE_OUTPUT
    Serial.println( message );
    Serial.flush();
#endif

    if( log_file_name.length() == 0 ) return;   // no log file found during setup

    File log = SD.open( log_file_name, FILE_WRITE );
    if( log )
    {
        log.println( message );
        log.close();
    }
}
