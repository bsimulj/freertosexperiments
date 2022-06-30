#include <stdint.h>

struct ModbusRegister{
    uint8_t Address;
    uint8_t FunctionCode;
    uint32_t dword;

    union Data
    {   uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        
    };
    
    

};
class Modbus{
    public:
        Modbus(){
            

        }


}

;
