
#ifndef TIMER_H_
#define TIMER_H_

#include <Arduino.h>

class Timer {
public:
   Timer();
   uint64_t getTickCount();
   uint64_t timeElapsed_us(uint64_t ticks_start, uint64_t ticks_final);
   uint64_t timeElapsed_us(uint64_t ticks_start);
   uint32_t timeElapsed_ms(uint64_t ticks_start);
   uint32_t timeElapsed_ms(uint64_t ticks_start, uint64_t ticks_final);
   float timeElapsed_s(uint64_t ticks_start);
};

#endif
