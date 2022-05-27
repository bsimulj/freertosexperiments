#include <stdio.h>
#include <pico/stdio.h>
#include "ConsoleInterface.hpp"
#include "MailBox.hpp"

extern stdio_driver_t stdio_usb;

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
    stdio_set_driver_enabled(&stdio_usb, true);
    stdio_usb_init();
    String S("console initialized");
    PrintResult(S);
}
void ConsoleInterface::PrintResult(String &s)
{
    s += "\n";
    printf(s.c_str());
    stdio_flush();
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
            stdio_flush();
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
        String S("pio");
        PrintResult(S);
        subset_ = "";
        subset_.trim();
        if (subsetCommand_ == "scanTime")
        {
            Message message;
            message.receiver = E_PROCESS_RECEIVE;
            message.cmd = E_READ_SCAN_TIME;

            MB.SendMessage(message);
        }
    }
    else if (subset_ != "")
    {
        String S("Unknown command");
        PrintResult(S);
        subset_ = "";
        subset_.trim();
    }
}
