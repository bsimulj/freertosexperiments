#include <stdint.h>
#include "ProcessImage.hpp"
#include "Arduino.h"
#include "Arduino_FreeRTOS.h"

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
    in_.b0 = PIND;
}

void ProcessImage::WriteOutputs()
{
    out_.x.di0 = !out_.x.di0;
    digitalWrite(LED_BUILTIN, out_.x.di0);
    scanTime_ms_ = static_cast<uint16_t>(millis() - previousScan_ms);
    previousScan_ms = millis();
}

void ProcessImage::Init()
{
    in_.b0 = 0;
    out_.b0 = 0;

    pinMode(LED_BUILTIN, OUTPUT);
}