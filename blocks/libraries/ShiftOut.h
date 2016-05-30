#ifndef SHIFTOUT_H
#define SHIFTOUT_H

#include <mbed.h>

/** A simple serial driver for a shift register that uses only three digital out pins leaving SPI and i2c free. 
 * ShiftOut can be configured for any size shift register but defaults to eight bits.
 */
class ShiftOut {

    public :
    
        /** Constructs a new ShiftOut with the given three pins.
         * @param clk - the pin to use for the shift register clock.
         * @param data - the pin to use for the shift register data line.
         * @param latch - the pin to use for the shift register latch.
         * @param registerCount - the number of registers in the shift register, defaults to eight.
         */
        ShiftOut(PinName clk, PinName data, PinName latch, int8_t registerCount = 0x08) {
            clkout = new DigitalOut(clk);
            dataout = new DigitalOut(data);
            latchout = new DigitalOut(latch);
            this->registerCount = registerCount;
        }
        
        ~ShiftOut() {
            delete clkout;
            delete dataout;
            delete latchout;
        }
        
        /** Writes the given integer to the shift register
         */
        void write(int data) {
            *latchout = 0;
            for (int i = registerCount - 1; i >=  0; i--) {
                *clkout = 0;
                *dataout = (data & (1 << i)) != 0;
                *clkout = 1;
            }
            *latchout = 1;
        }
         
    private :
        DigitalOut *clkout;
        DigitalOut *dataout;
        DigitalOut *latchout;
        int8_t registerCount;     
};

#endif