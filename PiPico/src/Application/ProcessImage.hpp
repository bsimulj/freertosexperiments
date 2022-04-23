#pragma once 

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


class ProcessImage
{
    public:
        ProcessImage();
        ~ProcessImage();
        void ReadInputs();
        void WriteOutputs();
        uint16_t GetScanTime_ms() const;
    private:
        void Init();
        ByteWrapper in_;
        ByteWrapper out_;
        uint16_t scanTime_ms_;
        uint32_t previousScan_ms;
};