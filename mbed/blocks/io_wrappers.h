/* 
 * Wrappers to make I/O functions available with C linkage. This allows C++
 * methods to be called from C code.
 *
 * Copyright 2009-2010 The MathWorks, Inc. */

#include <inttypes.h>
#include <stdio.h> /* for size_t */

int Serial_begin(long r);
int Serial_read(void);
void Serial_write(uint8_t * c, size_t s);
