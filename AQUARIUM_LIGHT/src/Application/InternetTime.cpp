#include "InternetTime.hpp"
#include "MailBox.hpp"
#include <cstring>
#include <nvs_flash.h>
#include <sstream>

#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <esp_log.h>
#include <esp_sntp.h>
#include <string.h>

void InternetTime::Init()
{
    wifiSid = "Simulj";
    wifiPass = "siimuljiii";

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_netif_init());
    tcpipNetworkStack = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(tcpipNetworkStack, "Aquarium Controler");
    assert(tcpipNetworkStack);

    wifi_init_config_t wifiInitCfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifiInitCfg));

    std::copy(wifiSid.begin(), wifiSid.end(), std::begin(wifi_config.sta.ssid));
    std::copy(wifiPass.begin(), wifiPass.end(), std::begin(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    SntpInit();
}

void InternetTime::SntpInit()
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_sync_interval(5000);
    sntp_init();
}

uint16_t InternetTime::IpFromNum(uint8_t addrIndex, uint32_t number)
{
    uint32_t temp = number;

    for (int i = 1; i < addrIndex; i++)
    {
        temp = temp >> 8;
    }
    return static_cast<uint8_t>(temp & 0x000000ff);
}

std::string InternetTime::IpToStr(uint32_t addr)
{
    std::ostringstream stream;
    stream << IpFromNum(1, addr) << "."
           << IpFromNum(2, addr) << "."
           << IpFromNum(3, addr) << "."
           << IpFromNum(4, addr);
    return stream.str();
}

void InternetTime::UnpdateTime()
{
    sntp_sync_status_t status = sntp_get_sync_status();

    switch (status)
    {
    case SNTP_SYNC_STATUS_RESET:
    {
        timeIsSet = false;
    }
    break;
    case SNTP_SYNC_STATUS_IN_PROGRESS:
    {
        timeIsSet = false;
    }
    break;
    case SNTP_SYNC_STATUS_COMPLETED:
    {
        timeIsSet = true;

        time(&now);

        // struct timezone {
        // 	int	tz_minuteswest;	/* minutes west of Greenwich */
        // 	int	tz_dsttime;	/* type of dst correction */
        // };
        // #define	DST_NONE	0	/* not on dst */
        // #define	DST_USA		1	/* USA style dst */
        // #define	DST_AUST	2	/* Australian style dst */
        // #define	DST_WET		3	/* Western European dst */
        // #define	DST_MET		4	/* Middle European dst */
        // #define	DST_EET		5	/* Eastern European dst */
        // #define	DST_CAN		6	/* Canada */


        // Set timezone to Belgrade Standard Time
        // setenv("TZ", "UTC+2", 1);
        // tzset();
        // localtime_r(&now, &timeinfo);
    }
    break;
    default:
        break;
    }
}

void InternetTime::Run()
{
    UnpdateTime();

    if (timeIsSet)
    {
        Message message;
        message.receiver = E_PROC_RECEIVE;
        message.cmd = E_UPDATE_TIME;
        message.value.actualTime = now;
        THREAD_SAFE.SendMessage(message);
    }
}

void InternetTime::PrintTime()
{
    std::ostringstream stream;
    stream << timeinfo.tm_hour
           << ":"
           << timeinfo.tm_min
           << std::endl;
    THREAD_SAFE.PrintMessage(stream.str());
}
bool InternetTime::Connect()
{
    ESP_ERROR_CHECK(esp_wifi_connect());
    return false;
}
void InternetTime::PrintIPInfo()
{
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(tcpipNetworkStack, &ip) == 0)
    {
        std::ostringstream stream;

        stream << "IP: \t" << IpToStr(ip.ip.addr) << std::endl;
        stream << "MASK: \t" << IpToStr(ip.netmask.addr) << std::endl;
        stream << "GW: \t" << IpToStr(ip.gw.addr) << std::endl;

        THREAD_SAFE.PrintMessage(stream.str());
    }
    else
    {
        THREAD_SAFE.PrintMessage("No ip conf\n");
    }
}
bool InternetTime::IsConnected()
{
    return false;
}

bool InternetTime::Disconnect()
{
    esp_wifi_disconnect();
    return false;
}