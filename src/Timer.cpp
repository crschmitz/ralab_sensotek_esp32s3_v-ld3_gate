#include "Timer.h"
#include "esp_timer.h"

Timer::Timer() {
}

uint64_t Timer::getTickCount() {
  return esp_timer_get_time();
}

uint64_t Timer::timeElapsed_us(uint64_t ticks_start, uint64_t ticks_final) {
  return ticks_final + ~ticks_start + 1;
}

uint64_t Timer::timeElapsed_us(uint64_t ticks_start) {
  return this->timeElapsed_us(ticks_start, this->getTickCount());
}

uint32_t Timer::timeElapsed_ms(uint64_t ticks_start, uint64_t ticks_final) {
  return this->timeElapsed_us(ticks_start, ticks_final) / 1000.0;
}

uint32_t Timer::timeElapsed_ms(uint64_t ticks_start) {
  return this->timeElapsed_ms(ticks_start, this->getTickCount());
}

float Timer::timeElapsed_s(uint64_t ticks_start) {
  return this->timeElapsed_ms(ticks_start) / 1000.0;
}

