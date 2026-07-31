#include "KJO_GSE_Display.h"

//
// GSEMU OLED display and log-file functions.
//
// scrollMessage() has moved to the shared KJO_Status_Display module
// (Report_Status()); this file now holds only Find_Available_File().
//

// --- Find_Available_File ------------------------------------------------------
//
// Scans the SD card root for the next unused log-file index and returns the
// file name string.  Files are named:  LOG_FILE_NAME_BASE + N + ".txt"
// (e.g. "Log_0.txt").  The returned string is ready to pass to SD.open().
//
String Find_Available_File()
{
    short  index = 0;
    String file_name = String( LOG_FILE_NAME_BASE ) + String( index ) + ".txt";

    while( SD.exists( file_name.c_str() ) )
    {
        index++;
        file_name = String( LOG_FILE_NAME_BASE ) + String( index ) + ".txt";
    }

    return file_name;
}
