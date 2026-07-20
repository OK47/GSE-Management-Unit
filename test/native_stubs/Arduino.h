#ifndef NATIVE_ARDUINO_STUB_H
#define NATIVE_ARDUINO_STUB_H

//
//    Ken Overton
//    Minimal Arduino.h stand-in for native (host) unit test builds only.
//    PlatformIO's "native" platform has no Arduino framework at all, so
//    any shared-library header that includes <Arduino.h> (e.g.
//    KJO_CAN_Command_Defs.h, for the `byte` type) needs *something* to
//    resolve that include against when built for env:native. This is
//    intentionally scoped to [env:native]'s build_flags -I path only
//    (see platformio.ini) so it can never shadow the real framework
//    Arduino.h used by the hardware environment.
//
//    Provides just the two symbols this test suite's dependencies
//    actually use: the `byte` type, and a monotonically-increasing
//    fake millis() (deterministic, not wall-clock -- sufficient for
//    CAN_Client's elapsed-time comparisons under test).
//

#include <cstdint>

typedef uint8_t byte;

inline unsigned long millis()
{
    static unsigned long fake_now = 0;
    return fake_now++;
}

#endif // NATIVE_ARDUINO_STUB_H
