#include "KJO_GSE_Display.h"

//
// GSEMU OLED display and log-file functions.
//

// --- scrollMessage ------------------------------------------------------------
//
// Adds 'message' to a circular buffer of MAX_LINES entries and redraws the
// entire scroll area.  Duplicate consecutive messages are suppressed.
// If delay_flag is true the function pauses 1 second so the user can read it.
//
void scrollMessage( Adafruit_SH1107 *display, String message, bool delay_flag )
{
    static String messageBuffer[MAX_LINES];
    static int    headIndex    = 0;
    static int    messageCount = 0;
    static String lastMessage  = "";

    int16_t  scrollWidth = display->width() - SCROLL_START_COL;

    // Only add the new message if it differs from the previous one
    if( message != lastMessage && message.length() > 0 )
    {
        lastMessage = message;

        // Truncate the message to fit the scroll area width
        String  truncatedMessage = message;
        int16_t x1, y1;
        uint16_t w, h;
        display->setTextSize( TEXT_SIZE );
        display->getTextBounds( truncatedMessage, 0, 0, &x1, &y1, &w, &h );

        while( w > scrollWidth && truncatedMessage.length() > 0 )
        {
            truncatedMessage = truncatedMessage.substring( 0, truncatedMessage.length() - 1 );
            display->getTextBounds( truncatedMessage, 0, 0, &x1, &y1, &w, &h );
        }

        // Insert at the head of the circular buffer (newest message at top)
        headIndex = (headIndex - 1 + MAX_LINES) % MAX_LINES;
        messageBuffer[headIndex] = truncatedMessage;
        if( messageCount < MAX_LINES ) messageCount++;

        if( delay_flag ) { delay( 1000 ); }
    }

    // Redraw the scroll area
    display->fillRect( SCROLL_START_COL, 0, scrollWidth, display->height(), SH110X_BLACK );
    display->setTextSize( TEXT_SIZE );
    display->setTextColor( SH110X_WHITE );

    for( int i = 0; i < messageCount; i++ )
    {
        int bufferIndex = (headIndex + i) % MAX_LINES;
        display->setCursor( SCROLL_START_COL, i * CHAR_HEIGHT );
        display->print( messageBuffer[bufferIndex] );
    }

    display->display();
}

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
