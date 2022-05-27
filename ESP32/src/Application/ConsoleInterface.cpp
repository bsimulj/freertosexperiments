#include "ConsoleInterface.hpp"
#include "MailBox.hpp"
#include <esp_console.h>
#include <linenoise/linenoise.h>
#include <sstream>
#include <stdio.h>

ConsoleInterface::ConsoleInterface()
{
}

ConsoleInterface::~ConsoleInterface()
{
}

void ConsoleInterface::Init()
{
}

void ConsoleInterface::PrintResult(const std::string &s)
{
    printf(s.c_str());
    fflush(stdout);
}

void ConsoleInterface::ReadCommand()
{
    char str[60];
    std::stringstream stream;
    fgets(str, sizeof(str), stdin);
    fflush(stdin);
    stream << str;

    if (stream.str().length() != 0)
    {
        subsetCommand_ = "";
        subsetCommand_.shrink_to_fit();
        subsetArg0_ = "";
        subsetArg0_.shrink_to_fit();
        subsetArg1_ = "";
        subsetArg1_.shrink_to_fit();

        stream >> subset_;
        if (stream.fail())
        {
            return;
        }

        stream >> subsetCommand_;
        if (stream.fail())
        {
            return;
        }

        stream >> subsetArg0_;
        if (stream.fail())
        {
            return;
        }
        stream >> subsetArg1_;
        if (stream.fail())
        {
            return;
        }
    }
}

void ConsoleInterface::ProcessCommand()
{
    if (subset_ == "pio")
    {
        printf("pio \n");
        subset_ = "";
        subset_.shrink_to_fit();
        if (subsetCommand_ == "scanTime")
        {
            Message message;
            message.receiver = E_PROCESS_RECEIVE;
            message.cmd = E_READ_SCAN_TIME;

            MailBox::Instance().SendMessage(message);
        }
    }
    else if (subset_ != "")
    {
        printf("Unknown command\n");
        subset_ = "";
        subset_.shrink_to_fit();
    }
}