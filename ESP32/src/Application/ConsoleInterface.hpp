#pragma once
#include <iostream>
#include <string.h>
#include <Primitives/Singleton.hpp>

#define CONSOLE ConsoleInterface::Instance()

class ConsoleInterface : public Singleton<ConsoleInterface>
{
    friend class Singleton<ConsoleInterface>;
public:
    void ReadCommand();
    void ProcessCommand();
    void PrintResult(const std::string &s);
    void Init();

private:
    ConsoleInterface();
    ~ConsoleInterface();
    std::string subset_;
    std::string subsetCommand_;
    std::string subsetArg0_;
    std::string subsetArg1_;
};
