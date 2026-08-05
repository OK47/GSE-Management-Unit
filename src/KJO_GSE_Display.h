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
//    Find_Available_File() scans the SD card root for the next unused
//    log-file index and returns the bare file name (e.g., "Log_3.txt").
//    Files are written flat to the SD card root -- there is no folder
//    prefix (see KJO_Logging.h).
//

#include <Arduino.h>
#include <SD.h>
#include "KJO_Logging.h"    // LOG_FILE_NAME_BASE

// --- Function declarations ----------------------------------------------------

// Scan the SD card and return the file name of the next unused log file.
// Files are named: LOG_FILE_NAME_BASE + <index> + ".txt", written flat to
// the SD card root (no folder prefix).
String Find_Available_File();

#endif // KJO_GSE_DISPLAY_H
