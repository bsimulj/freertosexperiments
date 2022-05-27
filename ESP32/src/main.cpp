#include <stdio.h>
#include <sstream>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
//#include "esp_system.h"
//#include "esp_spi_flash.h"
#include "Application/ProcessImage.hpp"
#include "Application/ConsoleInterface.hpp"
#include "Application/MailBox.hpp"

extern "C"
{
  void app_main(void);
}

// define two Tasks for DigitalRead & AnalogRead
void Process(void *pvParameters);
void Console(void *pvParameters);
ProcessImage pio;
ConsoleInterface console;

// the setup function runs once when you press reset or power the board
void app_main(void)
{
  pio.Init();
  console.Init();
  MailBox::Instance().Init();
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
  for (;;) // A Task shall never return or exit.
  {

    if (MailBox::Instance().MessageAvailable() && (MailBox::Instance().CheckReceiver() == E_PROCESS_RECEIVE))
    {
      console.PrintResult("Process message available\n");
      Message message = MailBox::Instance().ReceiveMessage();
      switch (message.cmd)
      {
      case E_READ_SCAN_TIME:
      {
        message.receiver = E_CONSOLE_RECEIVE;
        message.cmd = E_READ_SCAN_TIME;
        message.value.scanTime_us = pio.GetScanTime_us();
        MailBox::Instance().SendMessage(message);
      }
      break;

      default:
        break;
      }
    }

    pio.ReadInputs();
    pio.WriteOutputs();
    
    taskYIELD();
    // vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Console(void *pvParameters __attribute__((unused))) // This is a Task.
{
  for (;;) // A Task shall never return or exit.
  {
    if (MailBox::Instance().MessageAvailable() && (MailBox::Instance().CheckReceiver() == E_CONSOLE_RECEIVE))
    {
      console.PrintResult("console message available\n");
      Message message = MailBox::Instance().ReceiveMessage();
      switch (message.cmd)
      {
      case E_READ_SCAN_TIME:
      {
        std::stringstream stream;
        stream << "Scan lenght " << message.value.scanTime_us << "us\n";
        console.PrintResult(stream.str());
      }
      break;

      default:
        break;
      }
    }

    console.ReadCommand();
    console.ProcessCommand();

    taskYIELD();
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    // #define configUSE_PREEMPTION			0 this parameter is modified from 1 to 0
  }
}
