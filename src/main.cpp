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
    - Adafruit ADS1015 12-bit ADC daughter board (I2C)  [LiPo monitor + AUX analog input]
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
#include "KJO_QR_Slave.h"           // GSEMU-side umbilical quick-release actuator
#include <Adafruit_ADS1X15.h>
#include "KJO_Analog.h"             // ADS1015 channel assignments, LiPo scale, AUX threshold
#include <Adafruit_MCP2515.h>
#include "KJO_CAN_Command_Defs.h"
#include "KJO_CAN_Client.h"

// --- Conditional serial console ----------------------------------------------
// Uncomment to enable Serial output for debugging.
#define SERIAL_CONSOLE_OUTPUT

// --- Platform identifier for this build --------------------------------------
constexpr uint8_t PLATFORM = GSEMU_PLATFORM;

// --- CAN bus configuration -----------------------------------------------------
// CS is wired to RX (D0); confirmed hardware pin, same convention used on
// EMU and LCMU for consistency across all three boards. This design polls
// parsePacket() from loop() and never calls onReceive(), so CAN_INT_PIN is
// not read by firmware -- it's wired for consistency with the other two
// boards and left available for future interrupt-driven use. GPIO 13 is
// also this board's onboard LED (LED_BUILTIN); nothing in this firmware
// drives that LED, so no conflict.
constexpr uint8_t CAN_CS_PIN  = 0;
constexpr uint8_t CAN_INT_PIN = 13;

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

// CAN bus controller (SPI, Adafruit MCP2515 FeatherWing)
Adafruit_MCP2515 CAN_Controller( CAN_CS_PIN );

// Thin adapter so KJO_CAN_Client (hardware-independent) can drive this
// project's own CAN_Controller instance.
class MCP2515_Transport : public CAN_Transport
{
    public:
        void send( uint32_t id, const uint8_t *data, size_t len ) override
        {
            CAN_Controller.beginPacket( id );
            CAN_Controller.write( data, len );
            CAN_Controller.endPacket();
        }

        int parsePacket() override { return CAN_Controller.parsePacket(); }

        uint32_t packetId() override { return CAN_Controller.packetId(); }

        void readInto( uint8_t *buf, size_t len ) override
        {
            CAN_Controller.readBytes( buf, len );
        }
};

MCP2515_Transport Transport;
CAN_Client        Can( Transport, CAN_NODE_GSEMU );
bool               emu_pinged  = false;   // set once EMU's boot-time CAN_PING is received
bool               lcmu_pinged = false;   // set once LCMU responds to our boot-time CAN_PING

constexpr unsigned long CAN_RESPONSE_TIMEOUT_MS = 500;   // bumped from 200ms: doubles as Check_Fill()'s per-attempt weight-poll timeout

// SD/RTC init status -- used for CAN_GET_HEALTH_STATUS
bool sd_ok  = false;   // set in setup(); used for CAN_GET_HEALTH_STATUS
bool rtc_ok = false;   // set in setup(); used for CAN_GET_HEALTH_STATUS

// Sends a CAN command frame to 'destination' and blocks (bounded by
// CAN_RESPONSE_TIMEOUT_MS) waiting for its response. Returns false on
// timeout (response_out is left untouched).
bool Send_CAN_Command( uint8_t destination, CAN_Command command, float param,
                        CAN_Response_Frame &response_out )
{
    CAN_Command_Frame frame;
    frame.command = command;
    frame.param   = param;

    CAN_Controller.beginPacket( CAN_Pack_ID( destination, CAN_NODE_GSEMU ) );
    CAN_Controller.write( (uint8_t *)&frame, sizeof( frame ) );
    CAN_Controller.endPacket();

    unsigned long start = millis();
    while( millis() - start < CAN_RESPONSE_TIMEOUT_MS )
    {
        int packet_size = CAN_Controller.parsePacket();
        if( packet_size <= 0 ) continue;

        uint32_t id = CAN_Controller.packetId();
        if( CAN_Unpack_Destination( id ) != CAN_NODE_GSEMU ) continue;
        if( CAN_Unpack_Source( id )      != destination )     continue;

        CAN_Controller.readBytes( (uint8_t *)&response_out, sizeof( response_out ) );
        if( response_out.command != command ) continue;   // stray/unrelated response

        return true;
    }

    return false;
}

// Defined in Task 4 (fill state machine). 'source' plumbed through
// Fill_Begin() so Task 5 can route its deferred tare reply back to
// whichever node requested BEGIN_FILL; unused for now.
void        Fill_Set_Target( float target_lbm );
bool        Fill_Begin( uint8_t source );
void        Fill_Abort();
Fill_Status Fill_Get_Status();

// Forward declaration needed here (ahead of its "Forward declarations"
// block further down) since Send_CAN_Response() below now logs every TX
// via Post_Log_Message(), whose full definition lives much later in the
// file, near setup().
void Post_Log_Message( String S );

// Human-readable command name for logging -- bounds-checked since a
// corrupted/noise frame could carry a command byte outside the defined
// enum range, and CAN_Command_Name[] is not itself bounds-checked.
static const char *CAN_Command_Name_Safe( CAN_Command command )
{
    return ( command <= CAN_MAX_COMMAND_CODE ) ? CAN_Command_Name[ command ] : "UNKNOWN";
}

static void Send_CAN_Response( uint8_t destination, CAN_Command command, bool status, float r_val )
{
    CAN_Response_Frame resp;
    resp.command = command;
    resp.status  = status;
    resp.r_val   = r_val;

    CAN_Controller.beginPacket( CAN_Pack_ID( destination, CAN_NODE_GSEMU ) );
    CAN_Controller.write( (uint8_t *)&resp, sizeof( resp ) );
    CAN_Controller.endPacket();

    Post_Log_Message( String( "[TX] " ) + CAN_Command_Name_Safe( command ) + " (" + String( command ) + ")"
                     + " to node " + String( destination ) + " | status: " + ( status ? "OK" : "FAIL" )
                     + " | r_val: " + String( r_val, 3 ) );
}

// Advances Fill_Begin()'s deferred tare reply and Check_Fill()'s
// recurring weight poll -- filled in fully in the next task. Declared
// here (empty) so this task's Check_CAN() conversion compiles and is
// independently reviewable before Fill_Begin()/Check_Fill() themselves
// change.
static void Advance_Pending_Fill_Operations() {}

void Check_CAN()
{
    CAN_Command_Frame frame;
    uint8_t            source;
    CAN_Poll_Result    result = Can.poll( frame, source );

    // Advance any pending outbound operation regardless of what poll()
    // just returned -- a timeout can occur even on a tick where no
    // frame arrived at all.
    Advance_Pending_Fill_Operations();

    if( result != CAN_POLL_INBOUND_REQUEST ) return;

    Post_Log_Message( String( "[RX] " ) + CAN_Command_Name_Safe( frame.command ) + " (" + String( frame.command ) + ")"
                     + " from node " + String( source ) + " | param: " + String( frame.param, 3 ) );

    switch( frame.command )
    {
        case CAN_SET_FILL_TARGET:
            Fill_Set_Target( frame.param );
            Send_CAN_Response( source, CAN_SET_FILL_TARGET, true, 0.0f );
            break;

        case CAN_BEGIN_FILL:
            Fill_Begin( source );   // deferred reply -- see Task 5
            break;

        case CAN_QUERY_FILL_STATUS:
            Send_CAN_Response( source, CAN_QUERY_FILL_STATUS, true, (float)Fill_Get_Status() );
            break;

        case CAN_ABORT_FILL:
            Fill_Abort();
            Send_CAN_Response( source, CAN_ABORT_FILL, true, 0.0f );
            break;

        case CAN_GET_HEALTH_STATUS:
        {
            uint16_t bits = 0;
            if( !sd_ok )                                        bits |= HEALTH_SD_INIT_FAIL;
            if( !rtc_ok )                                        bits |= HEALTH_RTC_INIT_FAIL;
            if( E_GPIO.digitalRead( GSEMU_LINK_SENSE_EIO ) == HIGH ) bits |= HEALTH_LINK_SENSE_LOST;
            // HEALTH_GSEMU_FILL_VALVE_FAIL intentionally always 0 -- no
            // fault detection implemented for the Fill valve in this plan.
            Send_CAN_Response( source, CAN_GET_HEALTH_STATUS, true, (float)bits );
            break;
        }

        case CAN_PING:
            if( source == CAN_NODE_EMU ) emu_pinged = true;
            Send_CAN_Response( source, CAN_PING, true, 0.0f );
            break;

        default:
            // TARE / GET_BATTERY_VOLTAGE / REPORT_CURRENT_WEIGHT /
            // BEGIN-STOP_WEIGHT_RECORDING / BEGIN-STOP_THRUST_RECORDING are
            // LCMU-side commands -- not handled by GSEMU.
            break;
    }
}

// Fill valve  --  servo-actuated, now driven autonomously by GSEMU's own
// CAN-orchestrated fill state machine (Task 4) rather than an EMU GPIO
// handshake. Hardware constants from KJO_Valve.h.
Valve Fill_Valve( FILL_VALVE,
                  FILL_VALVE_PWM, FILL_VALVE_PWM_OPEN, FILL_VALVE_PWM_CLOSE,
                  FILL_VALVE_ANALOG_PIN, FILL_VALVE_POS_OPEN, FILL_VALVE_POS_CLOSED,
                  FILL_VALVE_BALL_DIAMETER, FILL_VALVE_BORE_DIAMETER, FILL_VALVE_THROAT_DIAMETER,
                  &Servos );

// Umbilical quick-release servo  --  driven by EMU GPIO CMD edge (QR_Slave).
// ⚠ QR_SERVO_PWM_HOLD / QR_SERVO_PWM_OPEN are placeholders — calibrate before use.
// Hardware constants from KJO_GPIO.h.
QR_Slave QR_Release( QR_SERVO_PWM_CHANNEL,
                     QR_SERVO_PWM_HOLD, QR_SERVO_PWM_OPEN,
                     QR_SERVO_MOVE_MS,
                     &Servos,
                     &E_GPIO, QR_CMD_EIO, RELEASE_STATE_EIO );

// External ADS1015 12-bit ADC (I2C)
Adafruit_ADS1015 Analog_Inputs;

// Battery voltage display timer
long battery_display_ms = 0;

// AUX analog input previous digital state (for edge detection in Check_AUX_Input())
bool aux_prev_state = false;   // expected LOW at startup

// QR separation event: latched true once umbilical separation has been logged.
bool qr_sep_logged  = false;

// SD card log file
SdFile Log_file;
String log_file_name;

// --- Forward declarations -----------------------------------------------------
void  Test_Buzzer();
void  Beep( short count );
void  Check_Buttons();
void  Post_Log_Message( String S );
float GSEMU_Battery_Voltage();
void  Check_Battery();
void  Check_AUX_Input();
void  Log_Config();

// --- Fill state machine ---------------------------------------------------------
// Armed by EMU's CAN_BEGIN_FILL (after CAN_SET_FILL_TARGET has already
// stored the target). Runs entirely locally on GSEMU: tares LCMU, opens
// the Fill valve, polls LCMU's current weight against the stored target,
// and closes the valve itself the moment the target is reached -- no
// further authorization from EMU is needed to close. If LCMU stops
// responding to weight polls, the fill is auto-aborted (valve closed)
// after FILL_MAX_CONSECUTIVE_POLL_FAILURES consecutive misses, rather
// than leaving the valve open indefinitely. EMU separately polls
// Fill_Get_Status() via CAN_QUERY_FILL_STATUS at a much lower rate.
constexpr unsigned long FILL_POLL_INTERVAL_MS               = 50;
constexpr uint8_t       FILL_MAX_CONSECUTIVE_POLL_FAILURES  = 3;

float         fill_target_lbm         = 0.0f;
bool          fill_active             = false;
Fill_Status   fill_state              = FILL_STATUS_IN_PROGRESS;
unsigned long last_fill_poll_ms       = 0;
uint8_t       fill_consecutive_fails  = 0;

void Fill_Set_Target( float target_lbm )
{
    fill_target_lbm = target_lbm;
}

bool Fill_Begin( uint8_t source )
{
    (void)source;   // routed through for Task 5's deferred-reply chaining; unused for now

    if( fill_active ) return true;   // already running

    CAN_Response_Frame tare_resp;
    if( !Send_CAN_Command( CAN_NODE_LCMU, CAN_TARE, 0.0f, tare_resp ) )
    {
        Post_Log_Message( "[FILL] LCMU did not respond to tare -- fill not started." );
        return false;
    }

    Fill_Valve.open();
    fill_active            = true;
    fill_state              = FILL_STATUS_IN_PROGRESS;
    fill_consecutive_fails  = 0;
    last_fill_poll_ms       = millis();
    Post_Log_Message( String( "[FILL] Started -- target " ) + String( fill_target_lbm, 2 ) + " lbm" );
    return true;
}

void Fill_Abort()
{
    if( !fill_active ) return;

    Fill_Valve.close();
    fill_active = false;
    fill_state  = FILL_STATUS_ABORTED;
    Post_Log_Message( "[FILL] Aborted -- valve closed." );
}

Fill_Status Fill_Get_Status()
{
    return fill_state;
}

// Call every loop() iteration. Non-blocking: polls LCMU at
// FILL_POLL_INTERVAL_MS, closes the valve and marks complete once the
// target is reached. Auto-aborts after FILL_MAX_CONSECUTIVE_POLL_FAILURES
// consecutive missed polls rather than leaving the valve open forever.
void Check_Fill()
{
    if( !fill_active ) return;

    unsigned long now = millis();
    if( now - last_fill_poll_ms < FILL_POLL_INTERVAL_MS ) return;
    last_fill_poll_ms = now;

    CAN_Response_Frame weight_resp;
    if( !Send_CAN_Command( CAN_NODE_LCMU, CAN_REPORT_CURRENT_WEIGHT, 0.0f, weight_resp ) )
    {
        fill_consecutive_fails++;
        Post_Log_Message( String( "[FILL] LCMU did not respond to weight poll (" )
                         + String( fill_consecutive_fails ) + "/" + String( FILL_MAX_CONSECUTIVE_POLL_FAILURES ) + ")." );

        if( fill_consecutive_fails >= FILL_MAX_CONSECUTIVE_POLL_FAILURES )
        {
            Fill_Valve.close();
            fill_active = false;
            fill_state  = FILL_STATUS_ABORTED;
            Post_Log_Message( "[FILL] Aborted -- LCMU unreachable after 3 consecutive polls." );
        }
        return;   // try again next interval unless the threshold above just closed the valve
    }

    fill_consecutive_fails = 0;

    if( weight_resp.r_val >= fill_target_lbm )
    {
        Fill_Valve.close();
        fill_active = false;
        fill_state  = FILL_STATUS_COMPLETE;
        Post_Log_Message( String( "[FILL] Target reached at " ) + String( weight_resp.r_val, 2 )
                         + " lbm -- valve closed." );
    }
}

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
    // AUX_IO_1 (GSEMU_LINK_SENSE_EIO) is configured below, near CAN bring-up.
    // AUX_IO_2 is currently unused.
    // AUX_IO_3 (QR_CMD_EIO) and AUX_IO_4 (RELEASE_STATE_EIO) are
    // configured by QR_Release.begin() called below.
    // AUX_IO_5 (LAUNCH_ENABLE_EIO) and AUX_IO_6 (REMOTE_START_EIO) configured below.

    // -- SD card ---------------------------------------------------------------
    sd_ok = SD.begin( CARD_CS );
    if( !sd_ok )
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
    rtc_ok = RT_Clock.begin();
    if( !rtc_ok )
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

    // -- External ADC (ADS1015) ------------------------------------------------
    Analog_Inputs.begin();
    Analog_Inputs.setGain( GAIN_ONE );
    {
        float   bat_v   = GSEMU_Battery_Voltage();
        String  bat_msg = "Bat: " + String( bat_v, 2 ) + " V";
        scrollMessage( &Screen, bat_msg, true );
        Post_Log_Message( bat_msg );
    }
    battery_display_ms = millis();
    scrollMessage( &Screen, "Analog ready.", true );

    // -- Launch Enable input and Remote Start output ---------------------------
    // LAUNCH_ENABLE_EIO (I/O 5): INPUT_PULLUP — reads EMU remote-start enable signal.
    // REMOTE_START_EIO  (I/O 6): OUTPUT — drives engine start command to EMU.
    //   Driven LOW only when AUX A/D input is HIGH AND LAUNCH_ENABLE reads LOW.
    E_GPIO.pinMode(      LAUNCH_ENABLE_EIO, INPUT_PULLUP );
    E_GPIO.pinMode(      REMOTE_START_EIO,  OUTPUT       );
    {
        int16_t aux_raw     = Analog_Inputs.readADC_SingleEnded( AD_AUX_CHANNEL );
        aux_prev_state      = ( aux_raw >= AD_AUX_THRESHOLD );
        bool launch_enabled = ( E_GPIO.digitalRead( LAUNCH_ENABLE_EIO ) == LOW );
        E_GPIO.digitalWrite( REMOTE_START_EIO,
                             ( aux_prev_state && launch_enabled ) ? LOW : HIGH );
        Post_Log_Message( "AUX initial state: " + String( aux_prev_state ? "HIGH" : "LOW" ) );
        Post_Log_Message( "Launch Enable: "
                          + String( launch_enabled ? "ENABLED" : "DISABLED" ) );
    }
    scrollMessage( &Screen, "AUX ready.", true );

    // -- Servo wing ------------------------------------------------------------
    Servos.begin();
    Servos.setPWMFreq( SERVO_FREQUENCY );
    scrollMessage( &Screen, "Servos ready.", true );

    // -- Onboard ADC resolution ------------------------------------------------
    analogReadResolution( 12 );
    scrollMessage( &Screen, "Analog 12 bits.", true );

    // -- Fill valve (plain Valve: closes to known state) ------------------------
    Fill_Valve.begin();
    scrollMessage( &Screen, "Fill valve ready.", true );

    // -- Link-disconnect sense (repurposed from the old Fill-valve handshake) --
    E_GPIO.pinMode( GSEMU_LINK_SENSE_EIO, INPUT_PULLUP );

    // -- CAN bus -----------------------------------------------------------
    if( !CAN_Controller.begin( CAN_BAUDRATE ) )
    {
        Post_Log_Message( "[CAN] mcp.begin() FAILED" );
        scrollMessage( &Screen, "CAN init FAILED", true );
    }
    else
    {
        Post_Log_Message( "[CAN] ready" );
        scrollMessage( &Screen, "CAN ready.", true );
    }

    Post_Log_Message( "[CAN] Waiting for EMU ping and LCMU response..." );
    scrollMessage( &Screen, "CAN: waiting...", true );
    Post_Log_Message( String( "[TX] PING (" ) + String( CAN_PING ) + ") to node " + String( CAN_NODE_LCMU ) );
    Can.sendRequest( CAN_NODE_LCMU, CAN_PING, 0.0f );
    unsigned long last_lcmu_ping_ms = millis();
    while( !emu_pinged || !lcmu_pinged )
    {
        Check_CAN();   // services both roles: answers EMU's inbound ping,
                        // and advances/reports Can's pending LCMU ping

        if( !lcmu_pinged && Can.requestState( 1000 ) == CAN_REQUEST_TIMED_OUT )
        {
            Post_Log_Message( String( "[TX] PING (" ) + String( CAN_PING ) + ") to node " + String( CAN_NODE_LCMU ) );
            Can.sendRequest( CAN_NODE_LCMU, CAN_PING, 0.0f );
            last_lcmu_ping_ms = millis();
        }
        if( !lcmu_pinged && Can.requestState( 1000 ) == CAN_REQUEST_COMPLETE )
        {
            Post_Log_Message( String( "[RX] PING (" ) + String( CAN_PING ) + ") response from node " + String( CAN_NODE_LCMU ) );
            lcmu_pinged = true;
        }
    }
    Post_Log_Message( "[CAN] EMU answered, LCMU responded -- both confirmed present." );
    scrollMessage( &Screen, "CAN: 3 nodes up.", true );

    // -- QR release servo (QR_Slave: sets servo to hold, configures CMD pin) --
    QR_Release.begin();
    scrollMessage( &Screen, "QR servo ready.", true );

    scrollMessage( &Screen, "GSEMU ready.", true );
    Post_Log_Message( "GSEMU startup complete, version " + String( GSEMU_VERSION ) );
    Log_Config();
}

// -----------------------------------------------------------------------------
void loop()
{
    // Service timed relay outputs
    Buzzer.update();

    // Service QR release servo: follow CMD level, detect physical separation
    QR_Release.update();

    // Log umbilical separation event once when first confirmed
    if( QR_Release.isSeparated() && !qr_sep_logged )
    {
        Post_Log_Message( "QR: umbilical separated." );
        scrollMessage( &Screen, "QR: Released", false );
        qr_sep_logged = true;
    }

    // Check front-panel buttons (local control)
    Check_Buttons();

    // Display battery voltage every BATTERY_DISPLAY_INTERVAL_MS (non-blocking)
    Check_Battery();

    // Poll AUX analog input; drive inverted output and log on state change
    Check_AUX_Input();

    // Service CAN command dispatch (EMU -> GSEMU)
    Check_CAN();

    // Service autonomous fill state machine (tare, open, poll, close on target)
    Check_Fill();
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

    // Button A  --  hold-to-release: open QR latch while held, close when released.
    // Pressing Button A calls localRelease() — QR_Slave::update() then ignores
    // the CMD line and keeps the servo open for as long as the button is held.
    // Releasing Button A calls localHold() — servo returns to CMD-line following.
    if( curr_A == LOW && prev_A == HIGH )
    {
        // Falling edge: button pressed — engage local release
        Beep( 1 );
        scrollMessage( &Screen, "QR: Releasing", false );
        Post_Log_Message( "QR: Local release commanded." );
        QR_Release.localRelease();
    }
    if( curr_A == HIGH && prev_A == LOW )
    {
        // Rising edge: button released — return servo to hold
        QR_Release.localHold();
        Post_Log_Message( "QR: Local release complete." );
        scrollMessage( &Screen, "QR: Hold", false );
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
// Reads and returns the GSEMU 2S LiPo battery voltage in volts.
// Uses ADS1015 channel AD_LIPO_CHANNEL with GSEMU-specific divider scale.
float GSEMU_Battery_Voltage()
{
    int16_t raw = Analog_Inputs.readADC_SingleEnded( AD_LIPO_CHANNEL );
    return (float)raw * AD_BASE_SCALE * GSEMU_LIPO_SCALE;
}

// -----------------------------------------------------------------------------
// Called from loop(). Displays the battery voltage on the scroll display and
// logs it to the SD card every BATTERY_DISPLAY_INTERVAL_MS.  Non-blocking.
void Check_Battery()
{
    if( (long)( millis() - battery_display_ms ) < (long)BATTERY_DISPLAY_INTERVAL_MS ) return;
    battery_display_ms = millis();

    float  bat_v = GSEMU_Battery_Voltage();
    String msg   = "Bat: " + String( bat_v, 2 ) + " V";
    scrollMessage( &Screen, msg, false );
    Post_Log_Message( msg );
}

// -----------------------------------------------------------------------------
// Called from loop() on every iteration.
// Reads the AUX analog input (ADS1015 channel AD_AUX_CHANNEL) and thresholds it
// to a digital HIGH/LOW state.  Logs AUX input transitions.
//
// Remote Start logic (gated):
//   REMOTE_START_EIO is driven LOW only when BOTH:
//     - AUX input is HIGH (above AD_AUX_THRESHOLD)
//     - LAUNCH_ENABLE_EIO reads LOW (EMU has enabled remote start)
//   Otherwise REMOTE_START_EIO is driven HIGH (hold / no start).
//
void Check_AUX_Input()
{
    int16_t raw            = Analog_Inputs.readADC_SingleEnded( AD_AUX_CHANNEL );
    bool    curr_state     = ( raw >= AD_AUX_THRESHOLD );
    bool    launch_enabled = ( E_GPIO.digitalRead( LAUNCH_ENABLE_EIO ) == LOW );

    // Log AUX input state transitions
    if( curr_state != aux_prev_state )
    {
        String msg = String( "AUX: " )
                   + ( aux_prev_state ? "HIGH" : "LOW" )
                   + " -> "
                   + ( curr_state     ? "HIGH" : "LOW" );
        Post_Log_Message( msg );
        scrollMessage( &Screen, msg, false );
        aux_prev_state = curr_state;
    }

    // Drive Remote Start: LOW = start commanded; HIGH = hold.
    // Gated — only asserted when AUX input HIGH AND remote start is enabled.
    E_GPIO.digitalWrite( REMOTE_START_EIO,
                         ( curr_state && launch_enabled ) ? LOW : HIGH );
}

// -----------------------------------------------------------------------------
// Writes a human-readable configuration snapshot to the SD log at startup.
// Logs firmware version, RTC time, valve calibration endpoints, QR servo
// calibration, and battery voltage.
void Log_Config()
{
    DateTime now = RT_Clock.now();
    Post_Log_Message( "[CFG] ===== GSEMU startup configuration =====" );
    Post_Log_Message( "[CFG] Version:   " + String( GSEMU_VERSION ) );
    Post_Log_Message( String("[CFG] RTC time: ")
        + String(now.year())   + "-"
        + String(now.month())  + "-"
        + String(now.day())    + " "
        + String(now.hour())   + ":"
        + String(now.minute()) + ":"
        + String(now.second()) );
    Post_Log_Message( "[CFG] Battery:   " + String( GSEMU_Battery_Voltage(), 2 ) + " V" );

    // --- Fill valve calibration ----------------------------------------------
    Post_Log_Message( String("[CFG] Fill PWM open=") + String(FILL_VALVE_PWM_OPEN)
        + " close=" + String(FILL_VALVE_PWM_CLOSE)
        + " | Pot open=" + String(FILL_VALVE_POS_OPEN)
        + " closed="     + String(FILL_VALVE_POS_CLOSED) );
    Post_Log_Message( "[CFG] Valve dead-band: " + String( VALVE_POSITION_DEAD_BAND ) + " counts" );

    // --- QR servo calibration ------------------------------------------------
    Post_Log_Message( String("[CFG] QR servo hold=") + String(QR_SERVO_PWM_HOLD)
        + " open="     + String(QR_SERVO_PWM_OPEN)
        + " move_ms="  + String(QR_SERVO_MOVE_MS) );
    Post_Log_Message( "[CFG] ==========================================" );
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
