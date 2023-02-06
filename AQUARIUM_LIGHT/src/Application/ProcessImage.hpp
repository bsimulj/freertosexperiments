#pragma once

#include <stdio.h>
#include <Primitives/Singleton.hpp>

#define PIO ProcessImage::Instance()

union ByteWrapper
{
    struct Bits
    {
        uint8_t di0 : 1;
        uint8_t di1 : 1;
        uint8_t di2 : 1;
        uint8_t di3 : 1;
        uint8_t di4 : 1;
        uint8_t di5 : 1;
        uint8_t di6 : 1;
        uint8_t di7 : 1;
    } x;
    uint8_t b0;
};

struct BoardIO
{
    union Bits
    {
        // Left side     // GPIO :ChipIO : ADC   :RTCGPIO: TOUCH : REMARKS
        uint8_t d15 : 1; // 15   : 21    : 2_3   : 13    : 3     :
        uint8_t d2 : 1;  // 2    : 22    : 2_2   : 12    : 2     : buildInLed
        uint8_t d4 : 1;  // 4    : 24    : 2_0   : 10    : 0     :
        uint8_t d16 : 1; // 16   : 25    :
        uint8_t d17 : 1; // 17   : 27    :
        uint8_t d5 : 1;  // 5    : 34    :                       : JLink SS
        uint8_t d18 : 1; // 18   : 35    :                       : JLink SCK
        uint8_t d19 : 1; // 19   : 38    :                       : JLink MISO
        uint8_t d21 : 1; // 21   : 42    :                       : JLink SDA
        uint8_t rx0 : 1; // 3    : 40    :
        uint8_t tx0 : 1; // 1    : 41    :
        uint8_t d22 : 1; // 22   : 39    :                       : JLink SCL
        uint8_t d23 : 1; // 23   : 36    :                       : JLink MOSI
        // Right side    // GPIO :ChipIO : ADC   :RTCGPIO: TOUCH : REMARKS
        uint8_t d13 : 1; // 13   : 20    : 2_4   : 14    : 4     :
        uint8_t d12 : 1; // 12   : 18    : 2_5   : 15    : 5     :
        uint8_t d14 : 1; // 14   : 17    : 2_6   : 16    : 6     :
        uint8_t d27 : 1; // 27   : 16    : 2_7   : 17    : 7     :
        uint8_t d26 : 1; // 26   : 15    : 2_9   : 7     :       : DAC2
        uint8_t d25 : 1; // 25   : 14    : 2_8   : 6     :       : DAC1
        uint8_t d33 : 1; // 33   : 13    : 1_5   : 8     : 8     :
        uint8_t d32 : 1; // 32   : 12    : 1_4   : 9     : 9     :
        uint8_t d35 : 1; // 35   : 11    : 1_7   : 5     :       : Input only
        uint8_t d34 : 1; // 34   : 10    : 1_6   : 4     :       : Input only
        uint8_t vn : 1;  // 39   : 8     : 1_3   : 3     :       : Input only
        uint8_t vp : 1;  // 36   : 5     : 1_0   : 0     :       : Input only
        uint8_t en : 1;  //      : 9
    } x;
    uint32_t dw0;
};

class ProcessImage : public Singleton<ProcessImage>
{
    friend class Singleton<ProcessImage>;
public:
    void Init();
    void ReadInputs();
    void WriteOutputs();
    void MBProcessRun();
    BoardIO &IO();
    uint16_t GetScanTime_us() const;

private:
    ProcessImage();
    ~ProcessImage();
    BoardIO io_;
    uint32_t scanTime_us;
    uint64_t previousScan_us;
    bool forceOutEnabled;
    BoardIO setForcingMask;
    BoardIO resetForcingMask;
};