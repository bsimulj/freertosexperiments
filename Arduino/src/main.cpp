#include <Arduino_FreeRTOS.h>
#include <Arduino.h>
#include "Application/ProcessImage.hpp"
#include "Application/ConsoleInterface.hpp"
#include "Application/MailBox.hpp"

// define two Tasks for DigitalRead & AnalogRead
void Process(void *pvParameters);
void Console(void *pvParameters);
ProcessImage pio;
ConsoleInterface console;

// the setup function runs once when you press reset or power the board
void setup()
{

  console.Init();
  MailBox::Instance().Init();
  // Now set up two Tasks to run independently.

  xTaskCreate(
      Process, "Process" // A name just for humans
      ,
      256 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL // Parameters for the task
      ,
      2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL); // Task Handle

  // Now set up two Tasks to run independently.
  xTaskCreate(
      Console, "Console" // A name just for humans
      ,
      256 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL // Parameters for the task
      ,
      2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL); // Task Handle

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
  vTaskStartScheduler();
}

void loop()
{
  // Empty. Things are done in Tasks.
}

void Process(void *pvParameters __attribute__((unused))) // This is a Task.
{

  for (;;) // A Task shall never return or exit.
  {
    if (MailBox::Instance().MessageAvailable() && (MailBox::Instance().CheckReceiver() == E_PROCESS_RECEIVE))
    {
      Serial.println("Test za cika Pecu");
      Message message = MailBox::Instance().ReceiveMessage();
      switch (message.cmd)
      {
      case E_READ_SCAN_TIME:
      {
        message.receiver = E_CONSOLE_RECEIVE;
        message.cmd = E_READ_SCAN_TIME;
        message.value.scanTime_ms = pio.GetScanTime_ms();
        MailBox::Instance().SendMessage(message);
      }
      break;

      default:
        break;
      }
    }

    pio.ReadInputs();

/* Logic goes here ...
*/
    pio.WriteOutputs();
    vTaskDelay(0 / portTICK_PERIOD_MS);
  }
}

void Console(void *pvParameters __attribute__((unused))) // This is a Task.
{

  for (;;) // A Task shall never return or exit.
  {
    if (MailBox::Instance().MessageAvailable() && (MailBox::Instance().CheckReceiver() == E_CONSOLE_RECEIVE))
    {
      Serial.println("console message available");
      Message message = MailBox::Instance().ReceiveMessage();
      switch (message.cmd)
      {
      case E_READ_SCAN_TIME:
      {
        String S(message.value.scanTime_ms);
        S.concat(" ms");
        console.PrintResult(S);
      }
      break;

      default:
        break;
      }
    }

    console.ReadCommand();
    console.ProcessCommand();
  }
}
