#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct Message
{
    uint8_t receiver;
    uint8_t cmd;
    union MessageSpec
    {
        uint8_t digitalInputs;
        uint8_t digitalOutputs;
        uint16_t scanTime_us;
    }value;
};

enum MessageDeffinitions
{
    E_NO_RECEIVE = 0,
    E_PROCESS_RECEIVE = 1,
    E_CONSOLE_RECEIVE = 2,
    E_READ_SCAN_TIME = 0
};

class MailBox
{
public:
    static MailBox &Instance()
    {
        static MailBox instance;
        return instance;
    }
    void Init();
    bool MessageAvailable();
    uint8_t CheckReceiver();
    void SendMessage(Message message);
    Message ReceiveMessage();

protected:
    MailBox() {}

private:
    MailBox(MailBox const &);
    MailBox &operator=(MailBox const &);

    QueueHandle_t mailbox_;
    enum MessageDefinition
    {
        E_READ_SCAN_TIME = 0,
        E_FORCE_OUTPUTS = 1
    };
};