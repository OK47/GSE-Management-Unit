#include "KJO_Valve.h"
#include <math.h>

//
// Valve class method implementations.
//

// --- Local helper -------------------------------------------------------------

// Floating-point version of the built-in map() function.
static float mapf( float x, float in_min, float in_max, float out_min, float out_max )
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- Constructor --------------------------------------------------------------
//
// Parameter order: open value precedes close value for both servo and encoder.
// (Bug fix: original header had close/open; implementation and calls now consistent.)
//
Valve::Valve( byte use,
              byte PWM_channel,    int PWM_open,     int PWM_close,
              byte encoder_channel, int encoder_open, int encoder_close,
              float ball_OD, float bore_OD, float throat_ID,
              Adafruit_PWMServoDriver *Servos )
{
    _use = use;

    _Servo_def.channel   = PWM_channel;
    _Servo_def.open_val  = PWM_open;
    _Servo_def.close_val = PWM_close;

    _Pot_def.channel   = encoder_channel;
    _Pot_def.open_val  = encoder_open;
    _Pot_def.close_val = encoder_close;

    _Valve_def.ball_OD   = ball_OD;
    _Valve_def.bore_OD   = bore_OD;
    _Valve_def.throat_ID = throat_ID;

    _Servos = Servos;

    _raw_encoder_position  = 0;
    _raw_encoder_timestamp = 0;
    _block_after_start     = true;
}

// --- begin() -----------------------------------------------------------------
// Starts the servo driver, computes valve geometry angles, then closes the valve.
void Valve::begin()
{
    _Servos->begin();
    _Servos->setPWMFreq( SERVO_FREQUENCY );

    // Compute the rotation angles at which the ball begins restricting flow (a1)
    // and at which it fully blocks flow (a2).
    _Valve_def.L2 = sqrtf( sq(_Valve_def.ball_OD / 2) - sq(_Valve_def.bore_OD / 2) );
    _Valve_def.a1 = degrees( atan2f(  _Valve_def.L2,  _Valve_def.bore_OD / 2 )
                             - acosf( (_Valve_def.throat_ID / 2)
                                     / sqrtf( sq(_Valve_def.L2) + sq(_Valve_def.bore_OD / 2) ) ) );
    _Valve_def.a2 = degrees( atan2f(  _Valve_def.L2, -_Valve_def.bore_OD / 2 )
                             - acosf( (_Valve_def.throat_ID / 2)
                                     / sqrtf( sq(_Valve_def.L2) + sq(_Valve_def.bore_OD / 2) ) ) );
    _raw_encoder_position = positionSensorAverage();
    _raw_encoder_timestamp = millis();
    close();
    while( isMoving() )
    {
        delay( 10 );
    }
}

// --- Motion detection --------------------------------------------------------

// Returns true if the position encoder is still changing (valve is moving).
bool Valve::isMoving()
{
    bool r_val = true;
    long time_now = millis();
    if( abs(time_now - _raw_encoder_timestamp) < MOVE_DETECT_WAIT )
    {
        return r_val;
    }
    int new_position = positionSensorAverage();
    time_now = millis();

    if( abs(new_position - _raw_encoder_position) <= VALVE_MOVING_THRESHOLD )
    {
        r_val = false;
    }
    _raw_encoder_position = new_position;
    _raw_encoder_timestamp = time_now;
    return r_val;
}

bool Valve::isStopped() { return !isMoving(); }

// Returns true if the final position is within VALVE_POSITION_DEAD_BAND of target.
bool Valve::moveSucceeded()
{
    _raw_encoder_position = positionSensorAverage();
    _raw_encoder_timestamp = millis();
    return ( abs(_move_target_encoder - _raw_encoder_position) <= VALVE_POSITION_DEAD_BAND );
}

// --- Position getters ---------------------------------------------------------

int Valve::getEncoder() { return positionSensorAverage(); }

short Valve::getPositionPercent()
{
    _raw_encoder_position = positionSensorAverage();
    _raw_encoder_timestamp = millis();
    return encoderToPercent( _raw_encoder_position );
}

short Valve::getPositionEncoder()
{
    _raw_encoder_position = positionSensorAverage();
    _raw_encoder_timestamp = millis();
    return _raw_encoder_position;
}

float Valve::getCloseAngle() { return _Valve_def.a2; }
float Valve::getOpenAngle()  { return _Valve_def.a1; }

// --- Configuration setters/getters -------------------------------------------

void  Valve::setUse( short use )       { _use = use; }
short Valve::getUse()                  { return _use; }

void  Valve::setPWMChannel( short ch ) { _Servo_def.channel = ch; }
short Valve::getPWMChannel()           { return _Servo_def.channel; }

void Valve::setPWMLimits( int open_limit, int close_limit )
{
    _Servo_def.open_val  = open_limit;
    _Servo_def.close_val = close_limit;
}

int* Valve::getPWMLimits()
{
    static int arr[2];
    arr[0] = _Servo_def.open_val;
    arr[1] = _Servo_def.close_val;
    return arr;
}

void  Valve::setEncoderChannel( short ch ) { _Pot_def.channel = ch; }
short Valve::getEncoderChannel()           { return _Pot_def.channel; }

void Valve::setEncoderLimits( int open_position, int close_position )
{
    _Pot_def.open_val  = open_position;
    _Pot_def.close_val = close_position;
}

int* Valve::getEncoderLimits()
{
    static int arr[2];
    arr[0] = _Pot_def.open_val;
    arr[1] = _Pot_def.close_val;
    return arr;
}

// --- Open / close state checks -----------------------------------------------

bool Valve::isOpen()
{
    _raw_encoder_position = positionSensorAverage();
    _raw_encoder_timestamp = millis();
    return ( abs(_raw_encoder_position - _Pot_def.open_val) <= VALVE_POSITION_DEAD_BAND );
}

bool Valve::isClosed()
{
    _raw_encoder_position = positionSensorAverage();
    _raw_encoder_timestamp = millis();
    return ( abs(_raw_encoder_position - _Pot_def.close_val) <= VALVE_POSITION_DEAD_BAND );
}

// --- Movement ----------------------------------------------------------------

// Core (non-blocking) move command.  Converts percent to PWM and commands servo.
void Valve::moveValveToPosition( int move_to_percent )
{
    int PWM_target;
    move_to_percent = constrain( move_to_percent, 0, 100 );

    if( USE_MAPPING )
        PWM_target = percentToPWM( move_to_percent );
    else
        PWM_target = (int)round( mapf( move_to_percent, 0.0f, 100.0f,
                                       _Servo_def.close_val, _Servo_def.open_val ) );

    _raw_encoder_position = positionSensorAverage();
    _raw_encoder_timestamp = millis();
    _Servos->setPWM( _Servo_def.channel, 0, PWM_target );
    _move_target_encoder = percentToEncoder( move_to_percent );
    if( _block_after_start ) delay( MOVE_AFTER_START_WAIT );
}

void Valve::open()              { moveValveToPosition( 100 ); }
void Valve::openTo( int target ){ moveValveToPosition( target ); }
void Valve::close()             { moveValveToPosition( 0 ); }

// --- Post-start blocking control ---------------------------------------------

void Valve::enableBlockAfterStart()    { _block_after_start = true; }
void Valve::disableBlockAfterStart()   { _block_after_start = false; }
bool Valve::isBlockAfterStartEnabled() { return _block_after_start; }

// --- ADC averaging ------------------------------------------------------------

// Returns the average of VALVE_POSITION_AVERAGING_COUNT ADC reads on the pot channel.
int Valve::positionSensorAverage()
{
    int accumulator = 0;
    for( int i = 0; i < VALVE_POSITION_AVERAGING_COUNT; i++ )
        accumulator += analogRead( _Pot_def.channel );
    return (int)( (float)accumulator / (float)VALVE_POSITION_AVERAGING_COUNT + 0.5f );
}

// --- Unit conversions --------------------------------------------------------

// Converts percent open to the corresponding servo PWM count.
int Valve::percentToPWM( int percent )
{
    percent = constrain( percent, 0, 100 );
    if( !USE_MAPPING )
        return (int)round( mapf( percent, 0.0f, 100.0f, _Servo_def.close_val, _Servo_def.open_val ) );

    if( percent == 0 )   return _Servo_def.close_val;
    if( percent == 100 ) return _Servo_def.open_val;
    float angle = mapf( percent, 0.0f, 100.0f, _Valve_def.a2, _Valve_def.a1 );
    return (int)round( mapf( angle, 0.0f, 90.0f, _Servo_def.open_val, _Servo_def.close_val ) );
}

// Converts percent open to the corresponding pot ADC count.
int Valve::percentToEncoder( int percent )
{
    percent = constrain( percent, 0, 100 );
    if( !USE_MAPPING )
        return map( percent, 0, 100, _Pot_def.close_val, _Pot_def.open_val );

    if( percent == 0 )   return _Pot_def.close_val;
    if( percent == 100 ) return _Pot_def.open_val;
    float angle = mapf( percent, 0.0f, 100.0f, _Valve_def.a2, _Valve_def.a1 );
    return (int)round( mapf( angle, 0.0f, 90.0f, _Pot_def.close_val, _Pot_def.open_val ) );
}

// Converts percent open to rotation angle (degrees).
float Valve::percentToAngle( int percent )
{
    percent = constrain( percent, 0, 100 );
    if( !USE_MAPPING )
        return mapf( percent, 0.0f, 100.0f, 90.0f, 0.0f );

    if( percent == 0 )   return 90.0f;
    if( percent == 100 ) return  0.0f;
    return mapf( percent, 0.0f, 100.0f, _Valve_def.a2, _Valve_def.a1 );
}

// Converts rotation angle (degrees) to servo PWM count.
int Valve::angleToPWM( float angle_deg )
{
    return (int)round( mapf( constrain( angle_deg, 0.0f, 90.0f ),
                             0.0f, 90.0f,
                             _Servo_def.open_val, _Servo_def.close_val ) );
}

// Converts rotation angle (degrees) to pot ADC count.
int Valve::angleToEncoder( float angle_deg )
{
    return (int)round( mapf( constrain( angle_deg, 0.0f, 90.0f ),
                             0.0f, 90.0f,
                             _Pot_def.open_val, _Pot_def.close_val ) );
}

// Converts rotation angle (degrees) to percent open.
float Valve::angleToPercent( float angle_deg )
{
    angle_deg = constrain( angle_deg, 0.0f, 90.0f );
    if( !USE_MAPPING )
        return mapf( angle_deg, 0.0f, 90.0f, 100.0f, 0.0f );

    if( angle_deg == 0.0f  ) return 100.0f;
    if( angle_deg == 90.0f ) return   0.0f;
    return mapf( angle_deg, _Valve_def.a1, _Valve_def.a2, 100.0f, 0.0f );
}

// Converts a raw pot ADC count to percent open.
int Valve::encoderToPercent( int e_val )
{
    if( !USE_MAPPING )
        return (int)mapf( e_val, _Pot_def.close_val, _Pot_def.open_val, 0.0f, 100.0f );

    float angle = mapf( e_val, _Pot_def.close_val, _Pot_def.open_val, 90.0f, 0.0f );
    return (int)angleToPercent( angle );
}
