#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sstream>
#include <stdio.h>
//#include "esp_system.h"
//#include "esp_spi_flash.h"
#include "Application/ConsoleInterface.hpp"
#include "Application/MailBox.hpp"
#include "Application/ProcessImage.hpp"
#include "Application/GsmGrps.hpp"

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

  // Now set up two Tasks to run independently.
  xTaskCreate(
      Console, "Console" // A name just for humans
      ,
      4096 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL // Parameters for the task
      ,
      0 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL); // Task Handle
  
  // Now set up two Tasks to run independently.
  xTaskCreate(
      GsmGprsComm, "GsmGprsComm" // A name just for humans
      ,
      4096 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL // Parameters for the task
      ,
      0 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL); // Task Handle
}

void Process(void *pvParameters __attribute__((unused))) // This is a Task.
{
  bool test = false;
  uint64_t time = 0;
  for (;;) // A Task shall never return or exit.
  {
    MB.ProcessRun();
    PIO.ReadInputs();

    if (time < 100)
    {
      time = time + PIO.GetScanTime_us();
    }
    else
    {
      test = !test;
      time = 0;
    }

    PIO.IO().x.d4 = test;
    PIO.IO().x.d2 = test;
    PIO.WriteOutputs();

    taskYIELD();
    //  vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void Console(void *pvParameters __attribute__((unused))) // This is a Task.
{
  for (;;) // A Task shall never return or exit.
  {
    CONSOLE.MBConsoleRun();
    CONSOLE.ReadCommand();
    CONSOLE.ProcessCommand();

    taskYIELD();
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // #define configUSE_PREEMPTION			0 this parameter is modified from 1 to 0
  }
}

void GsmGprsComm(void *pvParameters __attribute__((unused)))
{
  for (;;) // A Task shall never return or exit.
  {
    gsmGprs.Run();
    taskYIELD();
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // #define configUSE_PREEMPTION			0 this parameter is modified from 1 to 0
  }
}
