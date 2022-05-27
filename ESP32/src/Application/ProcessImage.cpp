#include "ProcessImage.hpp"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

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

void ProcessImage::ReadInputs()
{
    io_.x.d13 = gpio_get_level(GPIO_NUM_13);
    io_.x.d12 = gpio_get_level(GPIO_NUM_12);
    io_.x.d14 = gpio_get_level(GPIO_NUM_14);
    io_.x.d27 = gpio_get_level(GPIO_NUM_27);
    io_.x.d26 = gpio_get_level(GPIO_NUM_26);
    io_.x.d25 = gpio_get_level(GPIO_NUM_25);
    io_.x.d33 = gpio_get_level(GPIO_NUM_33);
    io_.x.d32 = gpio_get_level(GPIO_NUM_32);
    io_.x.d35 = gpio_get_level(GPIO_NUM_35);
    io_.x.d34 = gpio_get_level(GPIO_NUM_34);
    io_.x.vn = gpio_get_level(GPIO_NUM_39);
    io_.x.vp = gpio_get_level(GPIO_NUM_36);
}

void ProcessImage::WriteOutputs()
{
    gpio_set_level(GPIO_NUM_15, io_.x.d15);

    io_.x.d2 = !io_.x.d2; // Toggle LED
    gpio_set_level(GPIO_NUM_2, io_.x.d2);

    gpio_set_level(GPIO_NUM_4, io_.x.d4);
    gpio_set_level(GPIO_NUM_16, io_.x.rx2);
    gpio_set_level(GPIO_NUM_17, io_.x.tx2);
    gpio_set_level(GPIO_NUM_5, io_.x.d5);
    gpio_set_level(GPIO_NUM_18, io_.x.d18);
    gpio_set_level(GPIO_NUM_19, io_.x.d19);
    gpio_set_level(GPIO_NUM_21, io_.x.d21);
    // gpio_set_level(GPIO_NUM_3,io_.x.rx0); Used for a console application
    // gpio_set_level(GPIO_NUM_1,io_.x.tx0);
    gpio_set_level(GPIO_NUM_22, io_.x.d22);
    gpio_set_level(GPIO_NUM_23, io_.x.d23);

    uint64_t timeStamp_us = static_cast<uint64_t>(esp_timer_get_time());
    scanTime_us = static_cast<uint16_t>(timeStamp_us - previousScan_us);
    previousScan_us = timeStamp_us;
}

void ProcessImage::Init()
{
    gpio_config_t conf = {};
    conf.intr_type = gpio_int_type_t::GPIO_INTR_DISABLE; // disable interupts

    // Inputs
    conf.mode = gpio_mode_t::GPIO_MODE_INPUT;
    uint64_t inputBitMask = 0;
    // inputBitMask = inputBitMask | (1ULL<<BitNumber); // this is now to configure which bits are inputs
    // GPIO is to used as BitNumber
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_13);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_12);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_14);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_27);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_26);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_25);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_33);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_32);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_35);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_34);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_39);
    inputBitMask = inputBitMask | (1ULL << GPIO_NUM_36);
    conf.pin_bit_mask = inputBitMask;
    gpio_config(&conf);

    // Outputs
    conf.mode = gpio_mode_t::GPIO_MODE_OUTPUT;
    uint64_t outputBitMask = 0;
    // outputBitMask = outputBitMask | (1ULL<<BitNumber); this is now to configure which bits are outputs
    // GPIO is to used as BitNumber
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_15);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_2);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_4);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_16);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_17);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_5);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_18);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_19);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_21);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_3);
    // outputBitMask = outputBitMask | (1ULL << GPIO_NUM_1);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_22);
    outputBitMask = outputBitMask | (1ULL << GPIO_NUM_23);

    conf.pin_bit_mask = outputBitMask;
    gpio_config(&conf);
    previousScan_us = 0;
}