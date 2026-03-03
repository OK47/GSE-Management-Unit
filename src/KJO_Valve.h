#ifndef KJO_VALVE_H
#define KJO_VALVE_H

//
//    Ken Overton
//    Valve subsystem - servo-actuated ball valves with potentiometer feedback
//
//    Covers servo-actuated ball valves including:
//      - Servo PWM drive configuration
//      - Potentiometer position-feedback configuration
//      - Ball-valve geometric parameters (used for angle-based position mapping)
//      - Servo mechanical travel geometry
//      - Motion detection thresholds
//      - The Valve base class
//
//    NOTE: The GSE Management Unit will define a new valve class (or classes)
//    that inherit from this Valve base class.  Per-valve hardware constants
//    for the GSE valves will be defined in that class's header.
//
//    BUG FIX (constructor parameter order):
//      The original KJO_Control_System.h declared the constructor with
//      (PWM_close, PWM_open) and (encoder_close, encoder_open) but the
//      implementation used (PWM_open, PWM_close) and (encoder_open, encoder_close).
//      Both the declaration and the main.cpp call have been corrected to match
//      the implementation: open value always precedes close value.
//

#include <Arduino.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <KJO_System_Config.h>          // USE_MAPPING, M_PI

// --- Valve index constants ---------------------------------------------------
// GSEMU only uses the Fill valve; index matches the enum in KJO_Command_Defs.h.
constexpr uint8_t FILL_VALVE = 0;

// --- Valve state constants ----------------------------------------------------
constexpr int8_t VALVE_OPEN    =  1;
constexpr int8_t VALVE_CLOSED  =  0;
constexpr int8_t VALVE_UNKNOWN = -1;

// --- Servo drive configuration ------------------------------------------------
// SERVO_FREQUENCY is intentionally set above the standard 50 Hz to compress the
// usable PWM count range into the lower portion of the servo travel.  This
// effectively improves position resolution over the ~1.1 turns of travel
// required for the application (5-turn servo, ServoCity 2000-0025-0504).
constexpr uint16_t SERVO_FREQUENCY         = 1100;  // Hz  (non-standard; see note above)

// Servo mechanical travel parameters (5-turn servo, ~1.1 turns used)
constexpr uint16_t SERVO_RANGE_MIN          =  500;  // us - pulse width at lower travel limit
constexpr uint16_t SERVO_RANGE_MAX          = 2500;  // us - pulse width at upper travel limit
constexpr uint16_t SERVO_FULL_RANGE_DEGREES = 1800;  // degrees of full mechanical travel (5 turns)
constexpr uint8_t  SERVO_PINION_TEETH       =   16;  // teeth on servo output pinion
constexpr uint8_t  VALVE_GEAR_TEETH         =   60;  // teeth on valve actuator gear
constexpr uint8_t  VALVE_TURN_DEGREES       =   90;  // degrees of valve travel (open -> close)

// --- Fill Valve hardware calibration -----------------------------------------
//  Servo wing PWM channel, open/close PWM counts, analog ADC pin,
//  open/close ADC counts, and ball-valve geometry.
//  Values match the EMU's KJO_Valve.h — same physical valve.
//
constexpr uint8_t  FILL_VALVE_PWM             =    4;
constexpr uint16_t FILL_VALVE_PWM_OPEN        = 2329;
constexpr uint16_t FILL_VALVE_PWM_CLOSE       = 3918;
constexpr uint8_t  FILL_VALVE_ANALOG_PIN      =   A4;
constexpr uint16_t FILL_VALVE_POS_OPEN        = 1310;
constexpr uint16_t FILL_VALVE_POS_CLOSED      = 3004;
constexpr float    FILL_VALVE_BALL_DIAMETER   = 0.520f;
constexpr float    FILL_VALVE_BORE_DIAMETER   = 0.275f;
constexpr float    FILL_VALVE_THROAT_DIAMETER = 0.275f;

// --- Motion detection thresholds ---------------------------------------------
constexpr uint8_t  STOPPED                     =    0;
constexpr uint8_t  MOVING                      =    1;
constexpr uint8_t  MOVE_SUCCEEDED              =    0;
constexpr uint8_t  MOVE_FAILED                 =    1;
constexpr uint8_t  MOVE_CONTINUING             =    2;
constexpr uint16_t MOVE_TIMEOUT                = 2500;  // ms  - maximum time allowed for a move
                                                         // (increased from 1000 — 5-turn servo through
                                                         //  60:16 gear needs ~1.5-2s for full stroke)
constexpr uint8_t  VALVE_POSITION_AVERAGING_COUNT =  4; // ADC reads averaged for position
constexpr uint8_t  VALVE_MOVING_THRESHOLD      =    2;  // counts - minimum delta to detect motion
constexpr uint8_t  VALVE_POSITION_DEAD_BAND    =   15;  // counts - position tolerance for open/close detection
constexpr uint8_t  MOVE_DETECT_WAIT            =   30;  // ms  - velocity sample interval in isMoving()
constexpr uint8_t  MOVE_AFTER_START_WAIT       =   60;  // ms  - blocking wait after setPWM() (when _block_after_start is true)

// --- Internal structures ------------------------------------------------------

// Servo drive and potentiometer limits for one axis
struct SP_Def
{
    byte channel;
    int  open_val;      // PWM or ADC count at fully open
    int  close_val;     // PWM or ADC count at fully closed
};

// Ball-valve geometric data (used to compute angle-based position mapping)
struct Valve_Geometry
{
    float ball_OD;      // Ball outer diameter (in)
    float bore_OD;      // Bore diameter (in)
    float throat_ID;    // Throat inner diameter (in)
    float L2;           // Half-chord of the bore cross-section (computed in begin())
    float a1;           // Rotation angle (deg) at which flow starts to be restricted
    float a2;           // Rotation angle (deg) at which flow is fully shut off
};

// --- Valve class --------------------------------------------------------------
//
// Controls a single servo-actuated ball valve.  All positions are expressed
// as a percentage of full open (0 = closed, 100 = fully open).
//
// This is the base class.  GSE-specific valve types inherit from Valve.
//
// BUG FIX: constructor parameter order corrected - open values precede close
// values for both PWM and encoder arguments (was reversed in original header).
//
class Valve
{
    public:
        Valve( byte use,
               byte PWM_channel,   int PWM_open,    int PWM_close,
               byte encoder_channel, int encoder_open, int encoder_close,
               float ball_OD, float bore_OD, float throat_ID,
               Adafruit_PWMServoDriver *Servos );
        void begin();

        // -- Movement -----------------------------------------------------------
        void open();                    // Move to 100 %
        void openTo( int target );      // Move to target % open
        void close();                   // Move to 0 %

        // -- State queries ------------------------------------------------------
        bool  isMoving();
        bool  isStopped();
        bool  isOpen();
        bool  isClosed();
        bool  moveSucceeded();

        // -- Configuration setters ----------------------------------------------
        void setUse( short use );
        void setPWMChannel( short channel );
        void setPWMLimits( int open_limit, int close_limit );
        void setEncoderChannel( short channel );
        void setEncoderLimits( int open_position, int close_position );

        // -- Configuration getters ----------------------------------------------
        short getUse();
        short getPWMChannel();
        int*  getPWMLimits();
        short getEncoderChannel();
        int*  getEncoderLimits();
        short getPositionPercent();
        short getPositionEncoder();
        float getCloseAngle();
        float getOpenAngle();
        int   getEncoder();

        // -- Post-start blocking control ---------------------------------------
        // When enabled (default), moveValveToPosition() delays MOVE_AFTER_START_WAIT ms
        // after setPWM() so the servo has time to begin responding before isMoving() samples.
        // Disable for simultaneous multi-valve operations where blocking is undesirable.
        void enableBlockAfterStart();
        void disableBlockAfterStart();
        bool isBlockAfterStartEnabled();

        // -- Unit conversion helpers --------------------------------------------
        int   percentToPWM( int percent );
        int   percentToEncoder( int percent );
        float percentToAngle( int percent );
        int   angleToPWM( float angle_deg );
        int   angleToEncoder( float angle_deg );
        float angleToPercent( float angle_deg );
        int   encoderToPercent( int encoder );

    private:
        byte   _use;
        int    _raw_encoder_position;
        long   _raw_encoder_timestamp;
        short  _move_target_encoder;
        bool   _block_after_start = true;   // if true, delay MOVE_AFTER_START_WAIT ms after setPWM()
        Adafruit_PWMServoDriver *_Servos;
        SP_Def          _Servo_def;
        SP_Def          _Pot_def;
        Valve_Geometry  _Valve_def;

        void moveValveToPosition( int target_percent );
        int  positionSensorAverage();
};

#endif // KJO_VALVE_H
