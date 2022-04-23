#pragma once
#include <Primitives/Singleton.hpp>
#include <FreeRTOS.h>
#include <queue.h>

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
        uint16_t scanTime_ms;
    }value;
};

enum MessageDeffinitions
{
    E_NO_RECEIVE = 0,
    E_PROCESS_RECEIVE = 1,
    E_CONSOLE_RECEIVE = 2,
    E_READ_SCAN_TIME = 0
};

class MailBox : public Singleton<MailBox>
{
    friend class Singleton<MailBox>;
    
public:
    void Init();
    bool MessageAvailable();
    uint8_t CheckReceiver();
    void SendMessage(Message message);
    Message ReceiveMessage();

private:
    MailBox(){};
    ~MailBox(){};
    QueueHandle_t mailbox_;
    enum MessageDefinition
    {
        E_READ_SCAN_TIME = 0,
        E_FORCE_OUTPUTS = 1
    };
};