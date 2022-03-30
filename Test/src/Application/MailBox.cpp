#include "MailBox.hpp"
#include "queue.h"

void MailBox::Init()
{
    mailbox_ = xQueueCreate(1, sizeof(Message));
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
    uint8_t receiver;
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
        xQueueSend(mailbox_, &message, (TickType_t)0); //portMAX_DELAY
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