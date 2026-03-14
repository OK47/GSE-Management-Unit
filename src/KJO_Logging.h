#ifndef KJO_LOGGING_H
#define KJO_LOGGING_H

//
//    Ken Overton
//    Data logging subsystem configuration
//
//    Hardware and file-system definitions for the SD card data logger.
//    Log files are sequentially numbered: Log_0.txt, Log_1.txt, etc.,
//    written directly to the root of the SD card.
//

#include <Arduino.h>

// --- SD card hardware ---------------------------------------------------------
constexpr uint8_t CARD_CS = 10;     // SPI chip select for the SD card

// --- Log file naming ----------------------------------------------------------
constexpr const char* LOG_FILE_NAME_BASE = "Log_";  // Base name; index + ".txt" appended → e.g. "Log_0.txt"

#endif // KJO_LOGGING_H
