#include <stdio.h>
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
    // initialize serial communication at 9600 bits per second:
    stdio_init_all();
}
void ConsoleInterface::PrintResult(String &s)
{
    s += "\n";
    printf(s.c_str());
}

void ConsoleInterface::ReadCommand()
{
    while (true)
    {
        char c = getchar();
        if ((c != '\n') && (c != '\r'))
        {
            inputBuffer_ += c;
        }
        else
        {
            break;
        }
    }

    if (inputBuffer_.length() != 0)
    {
        uint8_t phase = 0;
        subsetCommand_ = "";
        subsetCommand_.trim();
        subsetArg0_ = "";
        subsetArg0_.trim();
        subsetArg1_ = "";
        subsetArg1_.trim();

        for (uint8_t i = 0; i < inputBuffer_.length(); i++)
        {
            if ((inputBuffer_.charAt(i) == '\r') || (inputBuffer_.charAt(i) == '\n'))
            {
                break;
            }

            switch (phase)
            {
            case 0:
            {
                if (inputBuffer_.charAt(i) != ' ')
                {
                    subset_.concat(inputBuffer_.charAt(i));
                }
                else
                {
                    phase++;
                }
            }
            break;
            case 1:
            {
                if (inputBuffer_.charAt(i) != ' ')
                {
                    subsetCommand_.concat(inputBuffer_.charAt(i));
                }
                else
                {
                    phase++;
                }
            }
            break;
            case 2:
            {
                if (inputBuffer_.charAt(i) != ' ')
                {
                    subsetArg0_.concat(inputBuffer_.charAt(i));
                }
                else
                {
                    phase++;
                }
            }
            break;
            case 3:
            {
                if (inputBuffer_.charAt(i) != ' ')
                {
                    subsetArg1_.concat(inputBuffer_.charAt(i));
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
        inputBuffer_.trim();
    }
}

void ConsoleInterface::ProcessCommand()
{
    if (subset_ == "pio")
    {
        printf("pio \n");
        subset_ = "";
        subset_.trim();
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
        subset_.trim();
    }
}