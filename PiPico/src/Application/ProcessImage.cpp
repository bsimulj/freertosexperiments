#include <stdint.h>
#include "ProcessImage.hpp"
#include "Arduino.h"
#include "FreeRTOS.h"

ProcessImage::ProcessImage()
{
    Init();
}

uint16_t ProcessImage::GetScanTime_ms() const
{
    return scanTime_ms_;
}

ProcessImage::~ProcessImage()
{

}

void ProcessImage::ReadInputs()
{
    //in_.b0 = PIND;
}

void ProcessImage::WriteOutputs()
{
    out_.x.di0 = !out_.x.di0;
    digitalWrite(PICO_DEFAULT_LED_PIN, out_.x.di0);

    // Calculate scan lenght. Usefull to implement timers
    uint32_t timeStamp = to_ms_since_boot(get_absolute_time());
    scanTime_ms_ = static_cast<uint16_t>(timeStamp - previousScan_ms);
    previousScan_ms = timeStamp;
}

void ProcessImage::Init()
{
    in_.b0 = 0;
    out_.b0 = 0;

    pinMode(PICO_DEFAULT_LED_PIN, OUTPUT);
}
