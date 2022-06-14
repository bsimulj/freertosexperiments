#include "MailBox.hpp"
#include <Application/ConsoleInterface.hpp>
#include <Application/ProcessImage.hpp>
#include <freertos/queue.h>
#include <sstream>

MailBox &mbDebug = MB;

void MailBox::Init()
{
    mailbox_ = xQueueCreate(1, sizeof(Message));
}

void MailBox::ProcessRun()
{
    if (MB.MessageAvailable() && (MB.CheckReceiver() == E_PROCESS_RECEIVE))
    {
        ConsoleInterface::Instance().PrintResult("Process message available\n"); // Not a thread safe implementation. To be fixed later
        Message message = MailBox::Instance().ReceiveMessage();
        switch (message.cmd)
        {
        case MessageDefinition::E_READ_SCAN_TIME:
        {
            message.receiver = Receiver::E_CONSOLE_RECEIVE;
            message.cmd = E_READ_SCAN_TIME;
            message.value.scanTime_us = PIO.GetScanTime_us();
            MB.SendMessage(message);
        }
        break;

        default:
            break;
        }
    }
}

void MailBox::ConsoleRun()
{
    if (MB.MessageAvailable() && (MB.CheckReceiver() == E_CONSOLE_RECEIVE))
    {
        CONSOLE.PrintResult("console message available\n");
        Message message = MB.ReceiveMessage();
        switch (message.cmd)
        {
        case MessageDefinition::E_READ_SCAN_TIME:
        {
            std::stringstream stream;
            stream << "Scan lenght " << message.value.scanTime_us << "us\n";
            CONSOLE.PrintResult(stream.str());
        }
        break;
        case MessageDefinition::E_GSM_GPRS_COMMAND:
        {
            std::stringstream stream;
            stream << "Responce is: \n"; //<< message.stringValue
            CONSOLE.PrintResult(stream.str());
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
    if (mailbox_)
    {
        Message received;
        if (xQueuePeek(mailbox_, &received, (TickType_t)0))
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
        if (xQueuePeek(mailbox_, &received, (TickType_t)0))
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
        xQueueSend(mailbox_, &message, (TickType_t)0); // portMAX_DELAY
    }
}

Message MailBox::ReceiveMessage()
{
    Message received;
    received.receiver = E_NO_RECEIVE;
    if (mailbox_)
    {
        xQueueReceive(mailbox_, &received, (TickType_t)0);
    }
    return received;
}
