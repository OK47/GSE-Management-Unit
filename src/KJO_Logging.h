#ifndef KJO_LOGGING_H
#define KJO_LOGGING_H

//
//    Ken Overton
//    Data logging subsystem configuration
//
//    Hardware and file-system definitions for the SD card data logger.
//    Log files are sequentially numbered: Log_0.txt, Log_1.txt, etc.,
//    stored under the LOG_FILE_FOLDER directory on the SD card.
//

#include <Arduino.h>

// --- SD card hardware ---------------------------------------------------------
constexpr uint8_t CARD_CS = 10;     // SPI chip select for the SD card

// --- Log file naming ----------------------------------------------------------
constexpr const char* LOG_FILE_FOLDER    = "GSE_Logs/";  // Directory on SD card for log files
constexpr const char* LOG_FILE_NAME_BASE = "Log_";       // Base name; index + ".txt" is appended

#endif // KJO_LOGGING_H
