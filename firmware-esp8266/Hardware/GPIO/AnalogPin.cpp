#pragma once

#include "AnalogPin.hpp"


AnalogPin::AnalogPin(uint8_t Pin_, uint8_t gpio_functions, int16_t min_value, int16_t max_value) : 
        isPinEn(FALSE),
        pin(Pin_), 
        gpioFunctions(gpio_functions),
        minValue(min_value),
        maxValue(max_value) {}

void AnalogPin::initAnalogPin() {
    isPinEn = TRUE;
    switch (gpioFunctions) {
        case INPUT:
        case INPUT_PULLUP:          // aaaaaaaaaaaaaaaaaaaaaaa
        case INPUT_PULLDOWN_16:
    		pinMode((uint8_t)pin, gpioFunctions);
            break;
        case OUTPUT:
        case OUTPUT_OPEN_DRAIN:
    		pinMode((uint8_t)pin, gpioFunctions);
            break;
        default:
            FATAL_ERROR; // altri casi non gestiti, vedere Arduino.h
    }
}


int16_t AnalogPin::adcRead() {
    ASSERT(isPinEn);        // nessuno ha inizializzato il pin

    switch (gpioFunctions) {
        case INPUT:
        case INPUT_PULLUP:          // aaaaaaaaaaaaaaaaaaaaaaa
        case INPUT_PULLDOWN_16: 
            //return map(analogRead(pin), 0, 1023, minValue, maxValue);
            return analogRead(pin); //, 0, 1023, minValue, maxValue);
        case OUTPUT:
        case OUTPUT_OPEN_DRAIN:
            FATAL_ERROR;    // non si pò fare una analogRead su un pin inizializzato come output
            break;
        default:
            FATAL_ERROR; // altri casi non gestiti, vedere Arduino.h
    }
    FATAL_ERROR;
    return 0;
}

void AnalogPin::pwmWrite(int8_t val) {
    ASSERT(isPinEn);        // nessuno ha inizializzato il pin

    switch (gpioFunctions) {
        case INPUT:
        case INPUT_PULLUP:
        case INPUT_PULLDOWN_16:
            FATAL_ERROR;    // non si pò fare una analogWrite su un pin inizializzato come input
        case OUTPUT:
        case OUTPUT_OPEN_DRAIN:
            //Serial.println(val);
            //Serial.println(map(val, minValue, maxValue, 0, 255));
            analogWrite(pin, map(val, minValue, maxValue, 0, 255));
            break;
        default:
            FATAL_ERROR; // altri casi non gestiti, vedere Arduino.h
    }
}

