#pragma once

#include "../../ErrorsHandle/ProgrammingErrors.hpp"



class AnalogPin {
public:
    AnalogPin(uint8_t Pin_, uint8_t gpio_functions, int16_t min_value=0, int16_t max_value=100);

    void initAnalogPin();
    int16_t adcRead();
    void pwmWrite(int8_t val);


private:
    bool isPinEn;
    uint8_t pin;
    uint8_t gpioFunctions;
    int16_t minValue;
    int16_t maxValue;    
};

