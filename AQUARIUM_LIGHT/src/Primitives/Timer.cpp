#include "Timer.hpp"
#include "esp_timer.h"
#include <time.h>

Timer::Timer() : 
timerType(E_TIMER_ON_DELAY),
startTime(0),
elapsedTime(0),
setTime(0)
{
}

Timer::~Timer()
{
}

void Timer::SetTimerParams(TimerType type, uint32_t setTime)
{
    timerType = type;
    this->setTime = setTime;
}

void Timer::ResetTimer()
{
    startTime = static_cast<uint32_t>(esp_timer_get_time()/1000);
}

void Timer::ForceTimeOut()
{
    ResetTimer();
}

bool Timer::IsTimeOut()
{
    elapsedTime = static_cast<uint32_t>(esp_timer_get_time()/1000) - startTime;
    return elapsedTime >= setTime;
}

uint32_t Timer::GetElapsedTime()
{
    elapsedTime = static_cast<uint32_t>(esp_timer_get_time()/1000) - startTime;
    return elapsedTime;
}

uint32_t Timer::GetSetTime()
{
    return setTime;
}
