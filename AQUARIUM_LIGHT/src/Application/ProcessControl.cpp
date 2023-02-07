#include "ProcessControl.hpp"
#include "Application/MailBox.hpp"
#include "ProcessImage.hpp"
#include <esp_timer.h>
#include <sstream>

ProcessControl::ProcessControl(/* args */)
{
}

ProcessControl::~ProcessControl()
{
}

void ProcessControl::CheckEvents()
{
    if (THREAD_SAFE.MessageAvailable() && (THREAD_SAFE.CheckReceiver() == E_PROC_RECEIVE))
    {
        Message message;
        message = THREAD_SAFE.ReceiveMessage();

        switch (message.cmd)
        {
        case MessageDefinition::E_UPDATE_TIME:
        {
            time(&now);

            // Set timezone to Belgrade Standard Time
            setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
            tzset();
            localtime_r(&now, &actualTime);
        }
        break;
        case MessageDefinition::E_SET_HOUR:
        {
            startTime.tm_hour = message.value.timerValue;
        }
        break;
        case MessageDefinition::E_SET_MIN:
        {
            startTime.tm_min = message.value.timerValue;
        }
        break;
        case MessageDefinition::E_SET_PHASE1:
        {
            phase1Duration = message.value.timerValue;
        }
        break;
        case MessageDefinition::E_SET_PHASE2:
        {
            phase2Duration = message.value.timerValue;
        }
        break;
        case MessageDefinition::E_SET_PHASE3:
        {
            phase3Duration = message.value.timerValue;
        }
        break;
        default:
            break;
        }
    }
}

void ProcessControl::CheckStartingConditions()
{
    if ((startTime.tm_hour == actualTime.tm_hour) &&
        (startTime.tm_min == actualTime.tm_min))
    {
        timerActive = true;
    }
    else if (!timerActive)
    {
        timer.ResetTimer();
    }
}

void ProcessControl::Run()
{
    CheckEvents();
    CheckStartingConditions();

    if (timer.IsTimeOut())
    {
        timerActive = false;
    }

    uint32_t elapsed = timer.GetElapsedTime();
    uint32_t setTime = timer.GetSetTime();
    uint32_t duration = phase1Duration;

    bool centerLight = (elapsed >= duration) &&
                       (elapsed <= (setTime - duration));


    duration += phase2Duration;
    bool farLights = (elapsed >= duration) &&
                     (elapsed <= (setTime - duration));

    duration += phase3Duration;
    bool leftAdnRightLight = (elapsed >= duration) &&
                             (elapsed <= (setTime - duration));

    PIO.Output(ProcessImage::E_MAIN_LIGHT, timerActive);
    PIO.Output(ProcessImage::E_CENTER_LIGHT, centerLight);
    PIO.Output(ProcessImage::E_FAR_LEFT_LIGHT, farLights);
    PIO.Output(ProcessImage::E_FAR_RIGHT_LIGHT, farLights);
    PIO.Output(ProcessImage::E_LEFT_LIGHT, leftAdnRightLight);
    PIO.Output(ProcessImage::E_RIGHT_LIGHT, leftAdnRightLight);

    vTaskDelay(pdMS_TO_TICKS(1000));
}

void ProcessControl::Init()
{
    timer.SetTimerParams(E_TIMER_ON_DELAY, 50400000); // 43200000 12 h
    phase1Duration = 3600000;
    phase2Duration = 3600000;
    phase3Duration = 3600000;
    startTime.tm_hour = 11;
    startTime.tm_min = 00;
}
