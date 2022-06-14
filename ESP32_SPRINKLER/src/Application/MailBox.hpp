#pragma once
#include <Application/GsmGrps.hpp>
#include <Primitives/Singleton.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <string>

// Search for MB and replace with MailBox::Instance()
#define MB MailBox::Instance()

struct Message
{
    uint8_t receiver;
    uint8_t cmd;
    union MessageSpec
    {
        uint8_t digitalInputs;
        uint8_t digitalOutputs;
        uint16_t scanTime_us;
        GsmGrps::GsmGprsCommands command;
    } value;
    char stringValue[40];
};

enum Receiver
{
    E_NO_RECEIVE = 0,
    E_PROCESS_RECEIVE = 1,
    E_CONSOLE_RECEIVE = 2,
    E_GSM_GPRS_RECEIVE = 3
};

enum MessageDefinition
{
    E_READ_SCAN_TIME = 0,
    E_FORCE_OUTPUTS = 1,
    E_SEND_GPRS_COMMAND,
    E_GSM_GPRS_COMMAND
};

class MailBox : public Singleton<MailBox>
{
    friend class Singleton<MailBox>;

public:
    void Init();
    void ProcessRun();
    void ConsoleRun();
    bool MessageAvailable();

    uint8_t CheckReceiver();
    void SendMessage(Message message);
    Message ReceiveMessage();

private:
    MailBox(){};
    ~MailBox(){};
    QueueHandle_t mailbox_;
};
