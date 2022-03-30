#include "ConsoleInterface.hpp"
#include <Arduino_FreeRTOS.h>
#include <HardwareSerial.h>
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
    Serial.begin(9600);

    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
    }
}
void ConsoleInterface::PrintResult(String &s)
{
    Serial.println(s);
}

void ConsoleInterface::ReadCommand()
{

    if (Serial)
    {
        if (Serial.available())
        {
            inputBuffer_ = Serial.readString();
        }
        else if (inputBuffer_.length() != 0)
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
            /*
            Serial.println(inputBuffer_.c_str());
            Serial.print(subset_);
            Serial.print(subsetCommand_);
            Serial.print(subsetArg0_);
            Serial.println(subsetArg1_);
            */
            Serial.flush();

            inputBuffer_ = "";
            inputBuffer_.trim();
        }
    }
}

void ConsoleInterface::ProcessCommand()
{
    if (subset_ == "pio")
    {
        //Serial.println("pio");
        subset_ = "";
        subset_.trim();
        if (subsetCommand_ == "scanTime")
        {
            Message message;
            message.receiver = E_PROCESS_RECEIVE;
            message.cmd = E_READ_SCAN_TIME;

            MailBox::Instance().SendMessage(message);
        }

        Serial.flush();
    }
    else if (subset_ != "")
    {
        Serial.println("Unknown command");
        subset_ = "";
        subset_.trim();
        Serial.flush();
    }
}