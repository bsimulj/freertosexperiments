#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sstream>
#include <stdio.h>
//#include "esp_system.h"
//#include "esp_spi_flash.h"
#include "Application/ConsoleInterface.hpp"
#include "Application/GsmGrps.hpp"
#include "Application/MailBox.hpp"
#include "Application/ProcessImage.hpp"

extern "C"
{
  void app_main(void);
}

GsmGrps gsmGprs;

// define two Tasks for DigitalRead & AnalogRead
void Process(void *pvParameters);
void Console(void *pvParameters);
void GsmGprsComm(void *pvParameters);

// the setup function runs once when you press reset or power the board
void app_main(void)
{
  PIO.Init();
  CONSOLE.Init();
  MailBox::Instance().Init();
  gsmGprs.Init();
  //  Now set up two Tasks to run independently.

  xTaskCreate(
      Process, "Process" // A name just for humans
      ,
      2048 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL // Parameters for the task
      ,
      0 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL); // Task Handle
}

void Process(void *pvParameters __attribute__((unused))) // This is a Task.
{
  union Nozzles
  {
    struct Bits
    {
      uint8_t nozzle1 : 1;
      uint8_t nozzle2 : 1;
      uint8_t nozzle3 : 1;
    } x;
    uint8_t nozzles;
  } nozzles;

  bool ledState = true;

  bool nozzleButton = false;
  bool prevNozzleButton = false;
  uint32_t activeNozzleTimer = 0;
  uint8_t activeNozzles = 1;
  uint32_t setTime = 120000000;
  uint32_t actualTime = 0;
  uint8_t step = 1;
  for (;;) // A Task shall never return or exit.
  {
    MB.ProcessRun();
    PIO.ReadInputs();

    if (nozzleButton ^ PIO.IO().x.d14)
    {
      activeNozzleTimer = activeNozzleTimer + PIO.GetScanTime_us();
    }
    else
    {
      activeNozzleTimer = 0;
    }

    if (activeNozzleTimer >= 100000)
    {
      nozzleButton = PIO.IO().x.d14;
    }

    if ((prevNozzleButton == true) & (nozzleButton == false))
    {
      if (activeNozzles == 3)
      {
        activeNozzles = 1;
      }
      else
      {
        activeNozzles++;
      }
    }

    prevNozzleButton = nozzleButton;

    if (actualTime < setTime)
    {
      actualTime = actualTime + PIO.GetScanTime_us();
    }
    else
    {
      actualTime = 0;

      switch (step)
      {
      case 1:
      {
        ledState = !ledState;
        step = 2;
      }
      break;
      case 2:
      {
        ledState = !ledState;
        step = 3;
      }
      break;
      case 3:
      {
        ledState = !ledState;
        step = 1;
      }
      break;
      default:
      {
        step = 1;
      }
      break;
      }
    }

    nozzles.nozzles = 7;

    switch (step)
    {
    case 1:
    {
      if (activeNozzles >= 1)
      {
        nozzles.x.nozzle1 = false;
      }

      if (activeNozzles >= 2)
      {
        nozzles.x.nozzle2 = false;
      }

      if (activeNozzles >= 3)
      {
        nozzles.x.nozzle3 = false;
      }
    }
    break;
    case 2:
    {
      if (activeNozzles >= 1)
      {
        nozzles.x.nozzle2 = false;
      }

      if (activeNozzles >= 2)
      {
        nozzles.x.nozzle3 = false;
      }

      if (activeNozzles >= 3)
      {
        nozzles.x.nozzle1 = false;
      }
    }
    break;
    case 3:
    {
      if (activeNozzles >= 1)
      {
        nozzles.x.nozzle3 = false;
      }

      if (activeNozzles >= 2)
      {
        nozzles.x.nozzle1 = false;
      }

      if (activeNozzles >= 3)
      {
        nozzles.x.nozzle2 = false;
      }
    }
    break;
    }

    PIO.IO().x.d2 = ledState;

    PIO.IO().x.d4 = nozzles.x.nozzle1;
    PIO.IO().x.d18 = nozzles.x.nozzle2;
    PIO.IO().x.tx0 = nozzles.x.nozzle3;

    PIO.WriteOutputs();

    taskYIELD();
    // vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}