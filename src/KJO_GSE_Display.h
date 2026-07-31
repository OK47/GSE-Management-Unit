#ifndef KJO_GSE_DISPLAY_H
#define KJO_GSE_DISPLAY_H

//
//    Ken Overton
//    GSEMU OLED display subsystem
//
//    The SD card log-file finder used during startup. The scrolling
//    boot-status display (formerly scrollMessage() here) now lives in the
//    shared KJO_Status_Display module as Report_Status().
//
//    Find_Available_File() scans the SD card for the next unused log-file
//    index and returns the full path string (e.g., "GSE_Logs/Log_3.txt").
//

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <SD.h>
#include "KJO_Logging.h"    // LOG_FILE_FOLDER, LOG_FILE_NAME_BASE

// --- Function declarations ----------------------------------------------------

// Scan the SD card and return the path of the next unused log file.
// Files are named: LOG_FILE_FOLDER + LOG_FILE_NAME_BASE + <index> + ".txt"
String Find_Available_File();

#endif // KJO_GSE_DISPLAY_H
