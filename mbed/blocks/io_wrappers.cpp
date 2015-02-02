/* 
 * Wrappers to make I/O functions available with C linkage. This allows C++
 * methods to be called from C code.
 *
 * Copyright 2009-2010 The MathWorks, Inc. */

#ifndef MATLAB_MEX_FILE

#include <inttypes.h>
#include "WProgram.h"

extern "C" { 
    void __cxa_pure_virtual(void);
}

extern "C" int Serial_begin(long r) { Serial.begin(r); }
extern "C" int Serial_read(void) { return Serial.read(); }
extern "C" void Serial_write(uint8_t * c, size_t s) { Serial.write(c, s); }

#endif /* #ifndef MATLAB_MEX_FILE */


