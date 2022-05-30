#include "ConsoleInterface.hpp"
#include "MailBox.hpp"
#include <soc/rtc.h>
#include <esp_system.h>
#include <sstream>
#include <stdio.h>

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
        subset_.clear();
        PrintResult("pio \n");
        if (subsetCommand_ == "scanTime")
        {
            Message message;
            message.receiver = E_PROCESS_RECEIVE;
            message.cmd = E_READ_SCAN_TIME;

            MailBox::Instance().SendMessage(message);
        }
    }
    else if (subset_ == "mcu")
    {
        subset_.clear();
        PrintResult("mcu \n");
        if (subsetCommand_ == "freq")
        {
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
        if (subsetCommand_ == "freqSetLow")
        {
            rtc_cpu_freq_config_t conf;
            conf.source = rtc_cpu_freq_src_t::RTC_CPU_FREQ_SRC_XTAL;
            rtc_clk_cpu_freq_set_config_fast(&conf);
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
        if (subsetCommand_ == "freqSetHigh")
        {
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
    }
    else if (subset_ != "")
    {
        subset_.clear();
        PrintResult("Unknown command\n");
    }
}