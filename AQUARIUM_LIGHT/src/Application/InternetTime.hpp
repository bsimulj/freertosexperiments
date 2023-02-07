#pragma once
#pragma once

#define NETTIME InternetTime::Instance()

#include "Primitives/Singleton.hpp"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include <esp_wifi.h>
#include <esp_wifi_netif.h>

#include <string>

class InternetTime : public Singleton<InternetTime>
{
    friend class Singleton<InternetTime>;

public:
    void Run();
    void Init();

    bool Connect();
    bool Disconnect();
    bool IsConnected();

private:
    void PrintTime();
    void PrintIPInfo();
    void SntpInit();
    void UnpdateTime();
    bool timeIsSet;
    time_t now;
    tm timeinfo;
    uint16_t IpFromNum(uint8_t addrIndex, uint32_t number);
    std::string IpToStr(uint32_t addr);
    esp_netif_t *tcpipNetworkStack;
    wifi_config_t wifi_config;
    std::string wifiSid;
    std::string wifiPass;
};
