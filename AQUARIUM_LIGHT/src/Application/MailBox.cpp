#include "MailBox.hpp"
#include "ConsoleInterface.hpp"
#include <Application/ConsoleInterface.hpp>
#include <Application/ProcessImage.hpp>
#include <freertos/queue.h>
#include <sstream>

MailBox &mbDebug = THREAD_SAFE;

void MailBox::Init()
{
    mailbox_ = xQueueCreate(1, sizeof(Message));
    textMailbox_ = xQueueCreate(1, sizeof(TextMessage));
}

void MailBox::ConsoleRun()
{
    if (THREAD_SAFE.MessageAvailable() && (THREAD_SAFE.CheckReceiver() == E_CONSOLE_RECEIVE))
    {
        Message message = THREAD_SAFE.ReceiveMessage();
        switch (message.cmd)
        {
        case MessageDefinition::E_READ_SCAN_TIME:
        {
            std::stringstream stream;
            stream << "Scan lenght " << message.value.scanTime_us << "us\n";
            THREAD_SAFE.PrintMessage(stream.str());
        }
        break;

        default:
            break;
        }
    }
}

bool MailBox::MessageAvailable()
{
    bool available = false;
    if (mailbox_ && textMailbox_)
    {
        Message received;
        if (xQueuePeek(mailbox_, &received, pdMS_TO_TICKS(0)))
        {
            available = true;
        }
    }
    return available;
}

uint8_t MailBox::CheckReceiver()
{
    uint8_t receiver = 0;
    if (mailbox_)
    {
        Message received;
        if (xQueuePeek(mailbox_, &received, pdMS_TO_TICKS(0)))
        {
            receiver = received.receiver;
        }
    }
    return receiver;
}

void MailBox::SendMessage(Message message)
{
    if (mailbox_)
    {
        xQueueSend(mailbox_, &message, pdMS_TO_TICKS(0)); // portMAX_DELAY
    }
}

Message MailBox::ReceiveMessage()
{
    Message received;
    if (mailbox_)
    {
        xQueueReceive(mailbox_, &received, pdMS_TO_TICKS(0));
    }
    return received;
}

bool MailBox::TextMessageAvailable()
{
    bool available = false;
    if (textMailbox_)
    {
        TextMessage textReceived;
        if (xQueuePeek(textMailbox_, &textReceived, pdMS_TO_TICKS(0)))
        {
            available = true;
        }
    }
    return available;
}

void MailBox::PrintMessage(std::string text)
{
    TextMessage message;
    memset(message.textMessage, 0, sizeof(message.textMessage));
    text.copy(message.textMessage, text.length(), 0);

    if (textMailbox_)
    {
        xQueueSend(textMailbox_, &message, pdMS_TO_TICKS(100));
    }
}

std::string MailBox::ReceiveTextMessage()
{
    TextMessage message;
    memset(message.textMessage, 0, sizeof(message.textMessage));

    std::string string;
    std::ostringstream stream;

    if (textMailbox_)
    {
        xQueueReceive(textMailbox_, &message, pdMS_TO_TICKS(100));
        string = message.textMessage;
    }
    return string;
}
