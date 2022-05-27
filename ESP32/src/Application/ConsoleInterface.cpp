#include <stdio.h>
#include <esp_console.h>
#include <linenoise/linenoise.h>
#include "ConsoleInterface.hpp"
#include "MailBox.hpp"

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
    fgets(str,60,stdin);
    inputBuffer_.assign(str);

    if (inputBuffer_.length() != 0)
    {
        uint8_t phase = 0;
        subsetCommand_ = "";
        subsetCommand_.shrink_to_fit();
        subsetArg0_ = "";
        subsetArg0_.shrink_to_fit();
        subsetArg1_ = "";
        subsetArg1_.shrink_to_fit();

        for (uint8_t i = 0; i < inputBuffer_.length(); i++)
        {
            if ((inputBuffer_.at(i) == '\r') || (inputBuffer_.at(i) == '\n'))
            {
                break;
            }

            switch (phase)
            {
            case 0:
            {
                if (inputBuffer_.at(i) != ' ')
                {
                    subset_.append(1, inputBuffer_.at(i));
                }
                else
                {
                    phase++;
                }
            }
            break;
            case 1:
            {
                if (inputBuffer_.at(i) != ' ')
                {
                    subsetCommand_.append(1, inputBuffer_.at(i));
                }
                else
                {
                    phase++;
                }
            }
            break;
            case 2:
            {
                if (inputBuffer_.at(i) != ' ')
                {
                    subsetArg0_.append(1, inputBuffer_.at(i));
                }
                else
                {
                    phase++;
                }
            }
            break;
            case 3:
            {
                if (inputBuffer_.at(i) != ' ')
                {
                    subsetArg1_.append(1, inputBuffer_.at(i));
                }
                else
                {
                    phase++;
                }
            }
            break;

            default:
                break;
            }
        }

        inputBuffer_ = "";
        inputBuffer_.shrink_to_fit();
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