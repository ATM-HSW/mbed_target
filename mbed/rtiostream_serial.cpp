/*
 * File: rtiostream_serial.cpp
 * Copyright 2001-2014 The MathWorks, Inc.
 *           2014 Dr. Olaf Hagendorf, HS Wismar
 */

#include "mbed.h"

extern "C" {
#include "rtiostream.h"
}

#ifndef PIL_SPEED
#define PIL_SPEED 9600
#endif

/***************** VISIBLE FUNCTIONS ******************************************/

/* Function: rtIOStreamOpen =================================================
 * Abstract:
 *  Open the connection with the target.
 */
int rtIOStreamOpen(int argc, void * argv[])
{
    volatile int flushedData;

    /* Initialize Arduino */
    init();
    
#if defined(USBCON) 
    /* Activate Leonardo's USB stack */
    USBDevice.attach();

    while (!Serial) {
            ; // wait for serial port to connect.
    }
#endif
    
    Serial.begin(PIL_SPEED);

    /* Flush out the serial receive buffer when opening a connection. This
     * works around an issue we've noticed with Arduino at high baud rates.
     * At high baud rates (i.e. 115200), the Arduino is receiving an
     * initial byte of spurious data (0xF0 / 240) even though the host has
     * not transmitted this data! This may cause an issue for PIL and
     * External mode during the handshaking process.
     */
    while (Serial.available()) {
        flushedData = Serial.read();
    }

    return RTIOSTREAM_NO_ERROR;
}

/* Function: rtIOStreamSend =====================================================
 * Abstract:
 *  Sends the specified number of bytes on the serial line. Returns the number of
 *  bytes sent (if successful) or a negative value if an error occurred.
 */
int rtIOStreamSend(
    int          streamID,
    const void * src,
    size_t       size,
    size_t     * sizeSent)
{
    Serial.write( (const uint8_t *)src, (int16_t)size);
    *sizeSent = size;
    return RTIOSTREAM_NO_ERROR;
}

/* Function: rtIOStreamRecv ================================================
 * Abstract: receive data
 *
 */
int rtIOStreamRecv(
    int      streamID,
    void   * dst,
    size_t   size,
    size_t * sizeRecvd)
{
    int data;
    uint8_t *ptr = (uint8_t *)dst;

    *sizeRecvd=0U;

    while ((*sizeRecvd < size)) {
        data = Serial.read();
        if (data != -1) {
            *ptr++ = (uint8_t) data;
            (*sizeRecvd)++;
        }
    }
    return RTIOSTREAM_NO_ERROR;
}

/* Function: rtIOStreamClose ================================================
 * Abstract: close the connection.
 *
 */
int rtIOStreamClose(int streamID)
{
    return RTIOSTREAM_NO_ERROR;
}
