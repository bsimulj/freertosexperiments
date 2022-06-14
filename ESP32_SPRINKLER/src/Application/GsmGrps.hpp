#pragma once

class GsmGrps
{
public:
    GsmGrps(/* args */);
    ~GsmGrps();

    void Init();
    void Run();

    enum GsmGprsCommands
    {
        AT = 0,
        comm2
    };

private:
    /* data */
    enum InternalDefinitions
    {
        E_TEST_TXD = 17,
        E_TEST_RXD = 16,

        E_UART_PORT_NUM = 2,
        E_UART_BAUD_RATE = 9600,

        E_BUF_SIZE = 150
    };

    void MBGsmGprsCommRun();
    void SendAtCommand(GsmGprsCommands command);

};

