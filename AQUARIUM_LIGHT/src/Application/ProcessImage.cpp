#include "ProcessImage.hpp"
#include "ConsoleInterface.hpp"
#include <driver/gpio.h>

#include <esp_timer.h>

#include <Application/MailBox.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include <cstring>
#include <sstream>

ProcessImage::ProcessImage()
{
}

uint16_t ProcessImage::GetScanTime_us() const
{
    return scanTime_us;
}

ProcessImage::~ProcessImage()
{
}

void ProcessImage::MBPIORun()
{
    if (THREAD_SAFE.MessageAvailable() && (THREAD_SAFE.CheckReceiver() == E_PIO_RECEIVE))
    {
        Message message = MailBox::Instance().ReceiveMessage();
        switch (message.cmd)
        {
        case MessageDefinition::E_READ_SCAN_TIME:
        {
            message.receiver = Receiver::E_CONSOLE_RECEIVE;
            message.cmd = E_READ_SCAN_TIME;
            message.value.scanTime_us = PIO.GetScanTime_us();
            THREAD_SAFE.SendMessage(message);
        }
        break;
        case MessageDefinition::E_FORCE_OUTPUTS_ENABLE:
        {
            forceOutEnabled = message.value.forceOutEnable;
            THREAD_SAFE.PrintMessage("Force out enabled");
        }
        break;
        case MessageDefinition::E_FORCE_OUTPUTS:
        {
            outForcingMask[message.value.signal.signal] = message.value.signal.value;

            std::stringstream stream;
            stream << "Forced sig:" << message.value.signal.signal << " as:" << message.value.signal.value;
            THREAD_SAFE.PrintMessage(stream.str());
        }
        break;
        default:
            break;
        }
    }
}

void ProcessImage::ReadInputs()
{
    MBPIORun();
    // io_.x.d13 = gpio_get_level(GPIO_NUM_13);
    // io_.x.d12 = gpio_get_level(GPIO_NUM_12);
    // io_.x.d14 = gpio_get_level(GPIO_NUM_14);
    // io_.x.d27 = gpio_get_level(GPIO_NUM_27);
    // io_.x.d26 = gpio_get_level(GPIO_NUM_26);
    // io_.x.d25 = gpio_get_level(GPIO_NUM_25);
    // io_.x.d33 = gpio_get_level(GPIO_NUM_33);
    // io_.x.d32 = gpio_get_level(GPIO_NUM_32);
    // io_.x.d35 = gpio_get_level(GPIO_NUM_35);
    // io_.x.d34 = gpio_get_level(GPIO_NUM_34);
    // io_.x.vn = gpio_get_level(GPIO_NUM_39);
    // io_.x.vp = gpio_get_level(GPIO_NUM_36);
}
void ProcessImage::WriteOutputs()
{
    if (forceOutEnabled)
    {
        io_ = outForcingMask;
    }
    else
    {
        outForcingMask = io_;
    }

    gpio_set_level(GPIO_NUM_5, !io_[E_FAR_LEFT_LIGHT]);
    gpio_set_level(GPIO_NUM_17, !io_[E_LEFT_LIGHT]);
    gpio_set_level(GPIO_NUM_16, !io_[E_CENTER_LIGHT]);
    gpio_set_level(GPIO_NUM_4, !io_[E_RIGHT_LIGHT]);
    gpio_set_level(GPIO_NUM_2, !io_[E_FAR_RIGHT_LIGHT]);
    gpio_set_level(GPIO_NUM_15, !io_[E_MAIN_LIGHT]);

    uint64_t timeStamp_us = static_cast<uint64_t>(esp_timer_get_time());
    scanTime_us = static_cast<uint32_t>(timeStamp_us - previousScan_us);
    previousScan_us = timeStamp_us;
}

void ProcessImage::Init()
{
    io_.reset();
    outForcingMask.reset() = 0;

    previousScan_us = 0;

    gpio_config_t conf = {};
    conf.intr_type = gpio_int_type_t::GPIO_INTR_DISABLE; // disable interupts

    // Inputs
    // conf.mode = gpio_mode_t::GPIO_MODE_INPUT;
    // uint64_t inputBitMask = 0;
    // inputBitMask = inputBitMask | (1ULL<<BitNumber); // this is now to configure which bits are inputs
    // GPIO is to used as BitNumber
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_13);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_12);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_14);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_27);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_26);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_25);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_33);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_32);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_35);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_34);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_39);
    // inputBitMask = inputBitMask | (1ULL << GPIO_NUM_36);
    // conf.pin_bit_mask = inputBitMask;
    // gpio_config(&conf);
    // Outputs
    conf.mode = gpio_mode_t::GPIO_MODE_OUTPUT;
    conf.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_ENABLE;
    uint64_t outputBitMask = 0;
    // outputBitMask = outputBitMask | (1ULL<<BitNumber); this is now to configure which bits are outputs
    // GPIO is to used as BitNumber
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_15);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_2);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_4);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_16);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_17);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_5);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_18);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_5);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_17);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_16);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_4);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_2);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_15);

    conf.pin_bit_mask = outputBitMask;
    gpio_config(&conf);
}

void ProcessImage::Output(SignalDefinitions signal, bool state)
{
    io_[signal] = state;
}