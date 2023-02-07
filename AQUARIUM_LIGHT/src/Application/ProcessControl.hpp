#pragma once

#include "Primitives/Singleton.hpp"
#include "Primitives/Timer.hpp"
#include <time.h>

#define PROC ProcessControl::Instance()

class ProcessControl : public Singleton<ProcessControl>
{
    friend class Singleton<ProcessControl>;

public:
    void Run();
    void Init();

private:
    time_t now;
    tm actualTime;
    tm startTime;
    bool timerActive;
    uint32_t startTimeStamp;
    uint32_t phase1Duration;
    uint32_t phase2Duration;
    uint32_t phase3Duration;
    Timer timer;
    ProcessControl(/* args */);
    ~ProcessControl();
    void CheckEvents();
    void CheckStartingConditions();
};
