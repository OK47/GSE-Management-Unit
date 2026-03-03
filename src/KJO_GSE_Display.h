#ifndef KJO_GSE_DISPLAY_H
#define KJO_GSE_DISPLAY_H

//
//    Ken Overton
//    GSEMU OLED display subsystem
//
//    Scrolling message display for the Adafruit SH1107 128x64 OLED, and
//    the SD card log-file finder used during startup.
//
//    scrollMessage() maintains a circular buffer of the most recent MAX_LINES
//    messages and redraws the full display on each call.  Pass delay_flag=true
//    to pause ~1 second after posting so the user can read the message.
//
//    Find_Available_File() scans the SD card for the next unused log-file
//    index and returns the full path string (e.g., "GSE_Logs/Log_3.txt").
//

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <SD.h>
#include "KJO_Logging.h"    // LOG_FILE_FOLDER, LOG_FILE_NAME_BASE

// --- OLED display layout constants -------------------------------------------
constexpr uint8_t SCROLL_START_COL = 0;     // Left edge of the scroll area (pixels)
constexpr uint8_t TEXT_SIZE        = 1;     // Adafruit GFX text size multiplier
constexpr uint8_t CHAR_HEIGHT      = 8;     // Pixel height of font at TEXT_SIZE 1
constexpr uint8_t MAX_LINES        = 8;     // Circular-buffer depth (lines of history)

// --- Function declarations ----------------------------------------------------

// Post a message to the scrolling OLED display.
// If delay_flag is true, pauses 1 second after updating the display.
void scrollMessage( Adafruit_SH1107 *display, String message, bool delay_flag );

// Scan the SD card and return the path of the next unused log file.
// Files are named: LOG_FILE_FOLDER + LOG_FILE_NAME_BASE + <index> + ".txt"
String Find_Available_File();

#endif // KJO_GSE_DISPLAY_H
