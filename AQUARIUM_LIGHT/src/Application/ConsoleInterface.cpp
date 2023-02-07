#include "ConsoleInterface.hpp"
#include "MailBox.hpp"
#include <esp32/rom/rtc.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/xtensa_timer.h>
#include <soc/rtc.h>
#include <sstream>
#include <stdio.h>

#include <esp_log.h>

void ConsoleInterface::Init()
{
    subset_.clear();
    subsetCommand_.clear();
    subsetArg0_.clear();
    subsetArg1_.clear();
    fflush(stdin);
}

void ConsoleInterface::PrintResult(const std::string &s)
{
    printf(s.c_str());
    fflush(stdout);
}

void ConsoleInterface::ReadCommand()
{
    char str[60];
    memset(&str, 0, sizeof(str));
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
            message.receiver = E_PIO_RECEIVE;
            message.cmd = E_READ_SCAN_TIME;

            MailBox::Instance().SendMessage(message);
        }
        if (subsetCommand_ == "forceOutEn")
        {
            bool enabled = std::stoi(subsetArg0_) == 1;
            Message message;
            message.receiver = E_PIO_RECEIVE;
            message.cmd = E_FORCE_OUTPUTS_ENABLE;
            message.value.forceOutEnable = enabled;
            THREAD_SAFE.SendMessage(message);
        }
        if (subsetCommand_ == "forceOut")
        {
            uint8_t signal = std::stoi(subsetArg0_);
            bool value = std::stoi(subsetArg1_) == 1;
            Message message;
            message.receiver = E_PIO_RECEIVE;
            message.cmd = E_FORCE_OUTPUTS;
            message.value.signal.signal = signal;
            message.value.signal.value = value;
            THREAD_SAFE.SendMessage(message);
        }
    }
    else if (subset_ == "light")
    {
        subset_.clear();
        PrintResult("light \n");
        if (subsetCommand_ == "sethour")
        {
            uint32_t value = std::stoi(subsetArg0_);
            Message message;
            message.receiver = E_PROC_RECEIVE;
            message.cmd = E_SET_HOUR;
            message.value.timerValue = value;
            THREAD_SAFE.SendMessage(message);
        }
        if (subsetCommand_ == "setmin")
        {
            uint32_t value = std::stoi(subsetArg0_);
            Message message;
            message.receiver = E_PROC_RECEIVE;
            message.cmd = E_SET_MIN;
            message.value.timerValue = value;
            THREAD_SAFE.SendMessage(message);
        }
        if (subsetCommand_ == "phase1")
        {
            uint32_t value = std::stoi(subsetArg0_);
            Message message;
            message.receiver = E_PROC_RECEIVE;
            message.cmd = E_SET_PHASE1;
            message.value.timerValue = value;
            THREAD_SAFE.SendMessage(message);
        }
        if (subsetCommand_ == "phase2")
        {
            uint32_t value = std::stoi(subsetArg0_);
            Message message;
            message.receiver = E_PROC_RECEIVE;
            message.cmd = E_SET_PHASE2;
            message.value.timerValue = value;
            THREAD_SAFE.SendMessage(message);
        }
        if (subsetCommand_ == "phase3")
        {
            uint32_t value = std::stoi(subsetArg0_);
            Message message;
            message.receiver = E_PROC_RECEIVE;
            message.cmd = E_SET_PHASE3;
            message.value.timerValue = value;
            THREAD_SAFE.SendMessage(message);
        }
    }
    else if (subset_ == "mcu")
    {
        subset_.clear();
        PrintResult("mcu \n");
        if (subsetCommand_ == "freq")
        {
            subsetCommand_.clear();
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
        if (subsetCommand_ == "freqSetLow")
        {
            subsetCommand_.clear();
            setCpuFrequencyMhz(80);
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
        if (subsetCommand_ == "freqSetNormal")
        {
            subsetCommand_.clear();
            setCpuFrequencyMhz(160);
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
        if (subsetCommand_ == "freqSetHigh")
        {
            subsetCommand_.clear();
            setCpuFrequencyMhz(240);
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
    }
    else if (!subset_.empty())
    {
        subset_.clear();
        PrintResult("Unknown command\n");
    }
}

void ConsoleInterface::MBConsoleRun()
{
    if (THREAD_SAFE.TextMessageAvailable())
    {
        std::stringstream stream;
        stream << THREAD_SAFE.ReceiveTextMessage() << "\n";
        PrintResult(stream.str());
    }

    if (THREAD_SAFE.MessageAvailable() && (THREAD_SAFE.CheckReceiver() == E_CONSOLE_RECEIVE))
    {
        Message message;
        message = THREAD_SAFE.ReceiveMessage();

        switch (message.cmd)
        {
        case MessageDefinition::E_READ_SCAN_TIME:
        {
            std::stringstream stream;
            stream << "Scan lenght " << message.value.scanTime_us << "us\n";
            PrintResult(stream.str());
        }
        break;

        default:
            break;
        }
    }
}

bool ConsoleInterface::setCpuFrequencyMhz(uint32_t cpu_freq_mhz)
{
    rtc_cpu_freq_config_t conf = {};
    // Get current CPU clock configuration
    rtc_clk_cpu_freq_get_config(&conf);
    // Get configuration for the new CPU frequency
    rtc_clk_cpu_freq_mhz_to_config(cpu_freq_mhz, &conf);
    // Make the frequency change
    rtc_clk_cpu_freq_set_config_fast(&conf);
    uint32_t apb = 0;
    // New APB
    if (conf.freq_mhz >= 80)
    {
        apb = 80 * MHZ;
    }
    else
    {
        apb = (conf.source_freq_mhz * MHZ) / conf.div;
    }
    // Update APB Freq REG
    rtc_clk_apb_freq_update(apb);
    // Update esp_timer divisor
    // esp_timer_impl_update_apb_freq(apb / MHZ);
    ets_update_cpu_frequency(cpu_freq_mhz);
    return true;
}
