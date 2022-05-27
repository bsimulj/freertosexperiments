#pragma once
#include <iostream>
#include <string.h>

class ConsoleInterface
{
public:
    ConsoleInterface();
    ~ConsoleInterface();
    void ReadCommand();
    void ProcessCommand();
    void PrintResult(const std::string& s);
    void Init();

private:
    std::string inputBuffer_;
    std::string subset_;
    std::string subsetCommand_;
    std::string subsetArg0_;
    std::string subsetArg1_;

};


