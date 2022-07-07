#include <stdint.h>

struct ModbusRegister{
    uint8_t Address;
    uint8_t FunctionCode;
    

    union Data
    {   struct
    {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
    }b;
        uint32_t dword;       
    };
    
    

};
class Modbus{
    public:
        Modbus()
        {
        }
        void Run();
    private:
        void Int();

}

;
