
#include <Primitives/Singleton.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <string>

// Search for MB and replace with MailBox::Instance()
#define THREAD_SAFE MailBox::Instance()

enum Receiver
{
    E_NO_RECEIVE = 0,
    E_PIO_RECEIVE,
    E_PROC_RECEIVE,
    E_CONSOLE_RECEIVE
};

enum MessageDefinition
{
    E_READ_SCAN_TIME = 0,
    E_PRINT_MESSAGE,
    E_FORCE_OUTPUTS_ENABLE,
    E_FORCE_OUTPUTS,
    E_SET_HOUR,
    E_SET_MIN,
    E_SET_PHASE1,
    E_SET_PHASE2,
    E_SET_PHASE3,
    E_UPDATE_TIME
};

struct Message
{
    uint8_t receiver;
    uint8_t cmd;
    union MessageSpec
    {
        bool forceOutEnable;
        struct SignalDef
        {
            uint8_t signal;
            bool value;
        }signal;
        uint8_t digitalInputs;
        uint8_t digitalOutputs;
        uint16_t scanTime_us;
        size_t messageSize;
        time_t actualTime;
        uint32_t timerValue;
    } value;
};

struct TextMessage
{
    uint8_t receiver = E_CONSOLE_RECEIVE;
    uint8_t cmd = E_PRINT_MESSAGE;
    char textMessage[128];
};

class MailBox : public Singleton<MailBox>
{
    friend class Singleton<MailBox>;

public:
    void Init();
    void ConsoleRun();
    bool MessageAvailable();

    uint8_t CheckReceiver();
    bool TextMessageAvailable();
    void SendMessage(Message message);
    void PrintMessage(std::string text);
    Message ReceiveMessage();
    std::string ReceiveTextMessage();

private:
    MailBox(){};
    ~MailBox(){};
    QueueHandle_t mailbox_;
    QueueHandle_t textMailbox_;
};
