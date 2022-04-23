#pragma once

#include "Arduino.h"

class ConsoleInterface
{
public:
    ConsoleInterface();
    ~ConsoleInterface();
    void ReadCommand();
    void ProcessCommand();
    void PrintResult(String& s);
    void Init();

private:
    String inputBuffer_;
    String subset_;
    String subsetCommand_;
    String subsetArg0_;
    String subsetArg1_;

};


