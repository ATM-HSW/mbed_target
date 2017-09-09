//
// File: grt_main.cpp
//
// Target selection: mbed_grt.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "mbed.h"
#include "rtos.h"
#include "mbed_grt_custom.h"

extern void (*stepFctPtr)(void);
extern void (*initFctPtr)(void);
extern void (*termFctPtr)(void);

osThreadId step_id;
Thread step_1_thread;
Ticker step_1_ticker;
void step_thread()
{
  step_id = Thread::gettid();
  while (1) {
    Thread::signal_wait(0x1,osWaitForever);
    (*stepFctPtr)();
  }
}

void step_callback()
{
  osSignalSet(step_id, 0x1);
}

// with RTOS
int main(void)
{
  step_1_thread.start(callback(step_thread));
  step_1_ticker.attach_us(callback(step_callback), (timestamp_t)STEP_SIZE_US);

  // initialize model
  (*initFctPtr)();

  // main task waits forever
  Thread::wait(osWaitForever);

  // deinitialize model
  (*termFctPtr)();
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
