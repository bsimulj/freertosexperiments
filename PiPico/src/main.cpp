#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "Arduino.h"
#include "FreeRTOS.h"

#include "Application/ConsoleInterface.hpp"
#include "Application/ProcessImage.hpp"
#include "Application/MailBox.hpp"

void Process(void *pvParameters);
void Console(void *pvParameters);
ConsoleInterface console;
ProcessImage pio;
String buffer;

void setup()
{
    buffer = "Testiranje\n";
    console.Init();
    MailBox::Instance().Init();

    xTaskCreate(
        Process, "Process" // A name just for humans
        ,
        512 // This stack size can be checked & adjusted by reading the Stack Highwater
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
        512 // This stack size can be checked & adjusted by reading the Stack Highwater
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
}

int main()
{
    setup();
    while (true)
    {
        loop();
    }

    while (true)
    {
        /* do nothing */
    }

    return 0;
}

void Process(void *pvParameters __attribute__((unused))) // This is a Task.
{

    for (;;) // A Task shall never return or exit.
    {

        if (MailBox::Instance().MessageAvailable() && (MailBox::Instance().CheckReceiver() == E_PROCESS_RECEIVE))
        {
            printf("process message available \n");
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

        // Logic goes here ...
        pio.WriteOutputs();

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void Console(void *pvParameters __attribute__((unused))) // This is a Task.
{
    for (;;) // A Task shall never return or exit.
    {
        if (MailBox::Instance().MessageAvailable() && (MailBox::Instance().CheckReceiver() == E_CONSOLE_RECEIVE))
        {
            printf("console message available \n");
            Message message = MailBox::Instance().ReceiveMessage();
            switch (message.cmd)
            {
            case E_READ_SCAN_TIME:
            {
                String S("scan time: ");
                S += String(message.value.scanTime_ms);
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
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}