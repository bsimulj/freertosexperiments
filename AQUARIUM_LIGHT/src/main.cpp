#include "Application/ConsoleInterface.hpp"
#include "Application/InternetTime.hpp"
#include "Application/MailBox.hpp"
#include "Application/ProcessControl.hpp"
#include "Application/ProcessImage.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <sstream>
// #include <stdio.h>

extern "C"
{
  void app_main(void);
}

// define two Tasks for DigitalRead & AnalogRead
void Process(void *pvParameters);
void Console(void *pvParameters);
void InternetTime(void *pvParameters);

// the setup function runs once when you press reset or power the board
void app_main(void)
{
  PIO.Init();
  CONSOLE.Init();
  THREAD_SAFE.Init();
  PROC.Init();

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
      InternetTime, "InternetTime" // A name just for humans
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
  for (;;) // A Task shall never return or exit.
  {
    PIO.ReadInputs();
    PROC.Run();
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

void InternetTime(void *pvParameters __attribute__((unused))) // This is a Task.
{
  NETTIME.Init();
  vTaskDelay(pdMS_TO_TICKS(5000));
  NETTIME.Connect();
  vTaskDelay(pdMS_TO_TICKS(5000));
  for (;;) // A Task shall never return or exit.
  {
    NETTIME.Run();
    vTaskDelay(pdMS_TO_TICKS(5000));
    // taskYIELD();
    // #define configUSE_PREEMPTION			0 this parameter is modified from 1 to 0
  }
}
