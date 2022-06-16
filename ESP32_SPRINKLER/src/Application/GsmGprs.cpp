#include "GsmGrps.hpp"
#include <driver/uart.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <sstream>
#include <stdio.h>
#include <string.h>

#include "Application/ConsoleInterface.hpp"
#include "Application/MailBox.hpp"

GsmGrps::GsmGrps(/* args */)
{
}

GsmGrps::~GsmGrps()
{
}

void GsmGrps::Init()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 1,
        .source_clk = uart_sclk_t::UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(E_UART_PORT_NUM, E_BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(E_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(E_UART_PORT_NUM, E_TEST_TXD, E_TEST_RXD, -1, -1));
}

void GsmGrps::Run()
{
    MBGsmGprsCommRun();

    char buffer[40];
    memset(&buffer, 0, sizeof(buffer));

    int len = 0;
    len = uart_read_bytes(E_UART_PORT_NUM, &buffer, sizeof(buffer), 500 / portTICK_PERIOD_MS);
    uart_flush(E_UART_PORT_NUM);

    if (len == -1)
    {
        // Send message to gsm
        std::stringstream stream;
        stream << "GSM Receiving Error: " << len << "\n";
        CONSOLE.PrintResult(stream.str());
    }
    else if (len > 0)
    {
        Message message;
        message.receiver = E_CONSOLE_RECEIVE;
        message.cmd = E_GSM_GPRS_COMMAND;
        memcpy(message.stringValue, &buffer, sizeof(buffer));
        MB.SendMessage(message);
    }
}

void GsmGrps::MBGsmGprsCommRun()
{
    if (MB.MessageAvailable() && (MB.CheckReceiver() == E_GSM_GPRS_RECEIVE))
    {
        Message message = MB.ReceiveMessage();
        switch (message.cmd)
        {
        case MessageDefinition::E_GSM_GPRS_COMMAND:
        {
            // Send message to gsm
            std::stringstream stream;
            stream << "Gsm Gprs command " << message.value.command << " sent.\n";
            CONSOLE.PrintResult(stream.str());
            SendAtCommand(message.value.command);
        }
        break;

        default:
            break;
        }
    }
}

void GsmGrps::SendAtCommand(GsmGprsCommands command)
{
    std::string buffer;
    buffer.clear();
    switch (command)
    {
    case AT:
    {
        buffer += "AT";
        buffer += 0x0D;
        buffer += 0x0A;
        int result = uart_write_bytes(E_UART_PORT_NUM, buffer.c_str(), sizeof(buffer.c_str()));
        if (result == -1)
        {
            CONSOLE.PrintResult("GSM Sending Error\n");
        }
        else
        {
            std::stringstream stream;
            stream << "Message sent: " << result << " bytes\n";
            CONSOLE.PrintResult(stream.str());
        }
    }
    break;

    case comm2:
    {
        buffer += "AT+CNMI=2,2,0,0,0";
        buffer += 0x0D;
        buffer += 0x0A;
        uart_write_bytes(E_UART_PORT_NUM, buffer.c_str(), sizeof(buffer.c_str()));
    }
    break;

    default:
        break;
    }
}