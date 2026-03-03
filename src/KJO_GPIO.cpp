#include "KJO_GPIO.h"

//
// Relay class method implementations.
//

// Constructor  --  stores the GPIO expander pointer and pin number, initialises state.
Relay::Relay( Adafruit_MCP23X17 *ptr, short pin )
{
    _on_timer    = false;
    _open_time   = 0;
    _control_pin = pin;
    _E_GPIO_ptr  = ptr;
}

// Configures the pin as an output and ensures the relay starts de-energised.
void Relay::begin()
{
    _E_GPIO_ptr->pinMode( _control_pin, OUTPUT );
    _E_GPIO_ptr->digitalWrite( _control_pin, LOW );
    open();
}

// Returns true if the relay is currently energised (contacts closed).
bool Relay::isOn() { return _E_GPIO_ptr->digitalRead( _control_pin ); }

// Energise the relay continuously (no automatic shutoff).
void Relay::close() { _E_GPIO_ptr->digitalWrite( _control_pin, HIGH ); }

// Alias for close().
void Relay::on() { close(); }

// Energise the relay for 'duration' milliseconds, then de-energise automatically.
// Requires update() to be called regularly from the main loop.
void Relay::close( long duration )
{
    _E_GPIO_ptr->digitalWrite( _control_pin, HIGH );
    _on_timer  = true;
    _open_time = millis() + duration;
}

// Alias for close(duration).
void Relay::on( long duration ) { close( duration ); }

// Set the GPIO pin number used to control this relay.
void Relay::setControlPin( short pin ) { _control_pin = pin; }

// Return the GPIO pin number used to control this relay.
short Relay::getControlPin() { return _control_pin; }

// Return the scheduled shutoff time (millis() timestamp).
long Relay::getOnTime() { return _open_time; }

// Poll the timer and de-energise the relay when the on-duration has elapsed.
// Must be called regularly from the main loop when timed operation is in use.
void Relay::update()
{
    if( _on_timer && millis() >= _open_time )
    {
        _E_GPIO_ptr->digitalWrite( _control_pin, LOW );
        _on_timer  = false;
        _open_time = 0;
    }
}

// De-energise the relay immediately, regardless of any active timer.
void Relay::open()
{
    _E_GPIO_ptr->digitalWrite( _control_pin, LOW );
    _on_timer  = false;
    _open_time = 0;
}

// Alias for open().
void Relay::off() { open(); }
