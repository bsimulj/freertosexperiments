#pragma once
#include <iostream>
#include <string.h>
#include <esp_console.h>
#include <Primitives/Singleton.hpp>

#define CONSOLE ConsoleInterface::Instance()

class ConsoleInterface : public Singleton<ConsoleInterface>
{
    friend class Singleton<ConsoleInterface>;
public:
    void ReadCommand();
    void ProcessCommand();
    void PrintResult(const std::string &s);
    void MBConsoleRun();
    void Init();

private:
    ConsoleInterface(){};
    ~ConsoleInterface(){};
    bool setCpuFrequencyMhz(uint32_t cpu_freq_mhz);
    std::string subset_;
    std::string subsetCommand_;
    std::string subsetArg0_;
    std::string subsetArg1_;
    esp_console_repl_t *repl;
};
