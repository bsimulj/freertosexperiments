#include "ConsoleInterface.hpp"
#include "GsmGrps.hpp"
#include "MailBox.hpp"
#include <esp32/clk.h>
#include <esp32/rom/rtc.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/xtensa_timer.h>
#include <soc/rtc.h>
#include <sstream>
#include <stdio.h>

void ConsoleInterface::Init()
{
    subset_.clear();
    subsetCommand_.clear();
    subsetArg0_.clear();
    subsetArg1_.clear();
    fflush(stdin);
}

void ConsoleInterface::PrintResult(const std::string &s)
{
    printf(s.c_str());
    fflush(stdout);
}

void ConsoleInterface::ReadCommand()
{
    char str[60];
    memset(&str, 0, sizeof(str));
    std::stringstream stream;
    fgets(str, sizeof(str), stdin);
    stream << str;

    if (stream.str().length() != 0)
    {
        stream >> subset_;
        if (stream.fail())
        {
            return;
        }

        stream >> subsetCommand_;
        if (stream.fail())
        {
            return;
        }

        stream >> subsetArg0_;
        if (stream.fail())
        {
            return;
        }
        stream >> subsetArg1_;
        if (stream.fail())
        {
            return;
        }
    }
}

void ConsoleInterface::ProcessCommand()
{
    if (subset_ == "pio")
    {
        subset_.clear();
        PrintResult("pio \n");
        if (subsetCommand_ == "scanTime")
        {
            Message message;
            message.receiver = E_PROCESS_RECEIVE;
            message.cmd = E_READ_SCAN_TIME;

            MailBox::Instance().SendMessage(message);
        }
    }
    else if (subset_ == "mcu")
    {
        subset_.clear();
        PrintResult("mcu \n");
        if (subsetCommand_ == "freq")
        {
            subsetCommand_.clear();
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
        if (subsetCommand_ == "freqSetLow")
        {
            subsetCommand_.clear();
            setCpuFrequencyMhz(80);
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
        if (subsetCommand_ == "freqSetNormal")
        {
            subsetCommand_.clear();
            setCpuFrequencyMhz(160);
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
        if (subsetCommand_ == "freqSetHigh")
        {
            subsetCommand_.clear();
            setCpuFrequencyMhz(240);
            std::stringstream stream;
            stream << "Freq: " << ets_get_cpu_frequency() << "\n";
            PrintResult(stream.str());
        }
    }
    else if (subset_ == "gpsgprs")
    {
        subset_.clear();
        PrintResult("gpsgprs \n");
        if (subsetCommand_ == "send")
        {
            int arg = stoi(subsetArg0_);
            Message message;
            message.receiver = Receiver::E_GSM_GPRS_RECEIVE;
            message.cmd = MessageDefinition::E_GSM_GPRS_COMMAND;
            message.value.command = static_cast<GsmGrps::GsmGprsCommands>(arg);

            MailBox::Instance().SendMessage(message);
        }
    }
    else if (!subset_.empty())
    {
        subset_.clear();
        PrintResult("Unknown command\n");
    }
}

void ConsoleInterface::MBConsoleRun()
{
    if (MB.MessageAvailable() && (MB.CheckReceiver() == E_CONSOLE_RECEIVE))
    {
        Message message = MB.ReceiveMessage();
        switch (message.cmd)
        {
        case MessageDefinition::E_READ_SCAN_TIME:
        {
            std::stringstream stream;
            stream << "Scan lenght " << message.value.scanTime_us << "us\n";
            CONSOLE.PrintResult(stream.str());
        }
        break;
        case MessageDefinition::E_GSM_GPRS_COMMAND:
        {
            std::stringstream stream;
            stream << "Responce is: " << message.stringValue << "\n"; 
            CONSOLE.PrintResult(stream.str());
        }
        break;

        default:
            break;
        }
    }
}

bool ConsoleInterface::setCpuFrequencyMhz(uint32_t cpu_freq_mhz)
{
    rtc_cpu_freq_config_t conf = {};
    // Get current CPU clock configuration
    rtc_clk_cpu_freq_get_config(&conf);
    // Get configuration for the new CPU frequency
    rtc_clk_cpu_freq_mhz_to_config(cpu_freq_mhz, &conf);
    // Make the frequency change
    rtc_clk_cpu_freq_set_config_fast(&conf);
    uint32_t apb = 0;
    // New APB
    if (conf.freq_mhz >= 80)
    {
        apb = 80 * MHZ;
    }
    else
    {
        apb = (conf.source_freq_mhz * MHZ) / conf.div;
    }
    // Update APB Freq REG
    rtc_clk_apb_freq_update(apb);
    // Update esp_timer divisor
    // esp_timer_impl_update_apb_freq(apb / MHZ);
    ets_update_cpu_frequency(cpu_freq_mhz);
    return true;
}

/*
typedef struct apb_change_cb_s
{
    struct apb_change_cb_s *prev;
    struct apb_change_cb_s *next;
    void *arg;
    apb_change_cb_t cb;
} apb_change_t;

static apb_change_t *apb_change_callbacks = NULL;
static xSemaphoreHandle apb_change_lock = NULL;

static void initApbChangeCallback()
{
    static volatile bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        apb_change_lock = xSemaphoreCreateMutex();
        if (!apb_change_lock)
        {
            initialized = false;
        }
    }
}

static void triggerApbChangeCallback(apb_change_ev_t ev_type, uint32_t old_apb, uint32_t new_apb)
{
    initApbChangeCallback();
    xSemaphoreTake(apb_change_lock, portMAX_DELAY);
    apb_change_t *r = apb_change_callbacks;
    if (r != NULL)
    {
        if (ev_type == APB_BEFORE_CHANGE)
            while (r != NULL)
            {
                r->cb(r->arg, ev_type, old_apb, new_apb);
                r = r->next;
            }
        else
        { // run backwards through chain
            while (r->next != NULL)
                r = r->next; // find first added
            while (r != NULL)
            {
                r->cb(r->arg, ev_type, old_apb, new_apb);
                r = r->prev;
            }
        }
    }
    xSemaphoreGive(apb_change_lock);
}

bool addApbChangeCallback(void *arg, apb_change_cb_t cb)
{
    initApbChangeCallback();
    apb_change_t *c = (apb_change_t *)malloc(sizeof(apb_change_t));
    if (!c)
    {
        log_e("Callback Object Malloc Failed");
        return false;
    }
    c->next = NULL;
    c->prev = NULL;
    c->arg = arg;
    c->cb = cb;
    xSemaphoreTake(apb_change_lock, portMAX_DELAY);
    if (apb_change_callbacks == NULL)
    {
        apb_change_callbacks = c;
    }
    else
    {
        apb_change_t *r = apb_change_callbacks;
        // look for duplicate callbacks
        while ((r != NULL) && !((r->cb == cb) && (r->arg == arg)))
            r = r->next;
        if (r)
        {
            log_e("duplicate func=%08X arg=%08X", c->cb, c->arg);
            free(c);
            xSemaphoreGive(apb_change_lock);
            return false;
        }
        else
        {
            c->next = apb_change_callbacks;
            apb_change_callbacks->prev = c;
            apb_change_callbacks = c;
        }
    }
    xSemaphoreGive(apb_change_lock);
    return true;
}

bool removeApbChangeCallback(void *arg, apb_change_cb_t cb)
{
    initApbChangeCallback();
    xSemaphoreTake(apb_change_lock, portMAX_DELAY);
    apb_change_t *r = apb_change_callbacks;
    // look for matching callback
    while ((r != NULL) && !((r->cb == cb) && (r->arg == arg)))
        r = r->next;
    if (r == NULL)
    {
        log_e("not found func=%08X arg=%08X", cb, arg);
        xSemaphoreGive(apb_change_lock);
        return false;
    }
    else
    {
        // patch links
        if (r->prev)
            r->prev->next = r->next;
        else
        { // this is first link
            apb_change_callbacks = r->next;
        }
        if (r->next)
            r->next->prev = r->prev;
        free(r);
    }
    xSemaphoreGive(apb_change_lock);
    return true;
}

if (apb_change_callbacks)
{
    triggerApbChangeCallback(APB_BEFORE_CHANGE, capb, apb);
}
// Make the frequency change
rtc_clk_cpu_freq_set_config_fast(&conf);
if (capb != apb)
{
    // Update REF_TICK (uncomment if REF_TICK is different than 1MHz)
    // if(conf.freq_mhz < 80){
    //     ESP_REG(APB_CTRL_XTAL_TICK_CONF_REG) = conf.freq_mhz / (REF_CLK_FREQ / MHZ) - 1;
    //  }
    // Update APB Freq REG
    rtc_clk_apb_freq_update(apb);
    // Update esp_timer divisor
    esp_timer_impl_update_apb_freq(apb / MHZ);
}
// Update FreeRTOS Tick Divisor
#if CONFIG_IDF_TARGET_ESP32C3

#else
uint32_t fcpu = (conf.freq_mhz >= 80) ? (conf.freq_mhz * MHZ) : (apb);
_xt_tick_divisor = fcpu / XT_TICK_PER_SEC;
#endif
// Call peripheral functions after the APB change
if (apb_change_callbacks)
{
    triggerApbChangeCallback(APB_AFTER_CHANGE, capb, apb);
}
*/
