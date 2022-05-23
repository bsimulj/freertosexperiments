#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

struct ProcessOutputs
{
  union
  {
    struct
    {
      uint8_t nozzle0 : 1;
      uint8_t nozzle1 : 1;
      uint8_t nozzle2 : 1;
    } x; // see naming convention
    uint8_t byte;
  };
};

ProcessOutputs out_;

uint64_t previousTime = 0;
uint16_t increment = 0;
uint32_t setCycle = 10000;
uint32_t timerCycle = 0;
uint32_t oneSec = 0;
uint32_t nozzleSwitch = 0;
uint8_t load = 100;
bool ledIndication;
  int i = 0;

bool nozle1;
bool nozle2;
bool nozle3;

void setup()
{
  ledIndication = false;
  out_.byte = 0;
  pinMode(4, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
}

void loop()
{

  increment = millis() - previousTime;
  previousTime = millis();

  if (timerCycle <= setCycle)
  {
    timerCycle += static_cast<uint32_t>(increment);
  }
  else
  {
    timerCycle = 0;
    /*
    switch (load)
    {
    case 100:
    {
      nozzleSwitch += static_cast<uint32_t>(increment);

      if (nozzleSwitch > 3)
      {
        nozzleSwitch = 0;
        out_.byte = 0;
        load = 66;
      }
    }
    break;
    case 66:
    {
      nozzleSwitch += static_cast<uint32_t>(increment);

      if (nozzleSwitch > 3)
      {
        nozzleSwitch = 0;

        out_.byte < 1;
        if (out_.byte == 6)
        {
          out_.byte = 3;
          load = 33;
        }
      }
    }
    break;
    case 33:
    {
      nozzleSwitch += static_cast<uint32_t>(increment);

      if (nozzleSwitch > 3)
      {
        nozzleSwitch = 0;
        out_.byte < 1;
        if (out_.byte == 4)
        {
          out_.byte = 6;
          load = 100;
        }
      }
    }
    default:
      load = 100;
      break;
    }
    */
    switch (i)
    {
    case 0:
      /* code */
      out_.x.nozzle0 = true;
      out_.x.nozzle1 = false;
      out_.x.nozzle2 = false;
      break;
    case 1:
      /* code */
      out_.x.nozzle0 = false;
      out_.x.nozzle1 = true;
      out_.x.nozzle2 = false;
      break;
    case 2:
      /* code */
      out_.x.nozzle0 = false;
      out_.x.nozzle1 = false;
      out_.x.nozzle2 = true;
      break;
    default:
      break;
    }

    if (i < 3)
    {
      i++;
    }
    else
    {
      i = 0;
    }

    digitalWrite(4, out_.x.nozzle0);
    digitalWrite(18, out_.x.nozzle1);
    digitalWrite(1, out_.x.nozzle2);

    ledIndication = !ledIndication;
    digitalWrite(BUILTIN_LED, ledIndication);
  }
}