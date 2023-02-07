#pragma once

#include <stdio.h>

enum TimerType
{
    E_TIMER_ON_DELAY = 0,
    E_TIMER_OFF_DELAY,
    E_TIMER_EXTENDED_PULSE
};

class Timer
{
public:
    Timer(/* args */);
    ~Timer();

void SetTimerParams(TimerType type, uint32_t setTime);

    void ResetTimer();
    void ForceTimeOut();
    bool IsTimeOut();
    uint32_t GetElapsedTime();
    uint32_t GetSetTime();

private:
    TimerType timerType;
    uint32_t startTime;
    uint32_t elapsedTime;
    uint32_t setTime;
};
