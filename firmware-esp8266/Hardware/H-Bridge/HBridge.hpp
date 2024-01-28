#pragma once

#include "../../ErrorsHandle/ProgrammingErrors.hpp"
#include "../../GlobalDependacies.hpp"
#include "../GPIO/AnalogPin.cpp"
#include "../GPIO/GPIO.cpp"



class HBridge {
public:
    HBridge(AnalogPin *pwmPin, GPIO *dir) : PWMpin(pwmPin), dir(dir), isInit(false) {
        ASSERT_PTR(PWMpin);
        ASSERT_PTR(dir);
    }

    void initHW() {
        isInit = true;
        ASSERT_PTR(PWMpin);
        ASSERT_PTR(dir);
        PWMpin->initAnalogPin();
        dir->initGPIO();
    }

    void setSpeed(float u) {
        FATAL_ERROR_IF_FALSE(isInit); // initWH non chiamata
        if (u > 0) {
            setDirection(true);
            //uint8_t pwmDes = map((int16_t)u, -100, 100, 0, 100);
            uint8_t pwmDes = (uint8_t)u;
            PWMpin->pwmWrite(pwmDes);   // u è 0-100

        } else if (u < 0) {
            setDirection(false);
            u = -u;
            uint8_t pwmDes = (uint8_t)u;
            PWMpin->pwmWrite(pwmDes);   // u è 0-100

        } else {
            // stai fermo
            PWMpin->pwmWrite(0);
        }
    }


private:

    void setDirection(bool goForward) {
        FATAL_ERROR_IF_FALSE(isInit); // initWH non chiamata
        if (goForward) {
            dir->turnOff();
        } else {
            dir->turnOn();
        }
    }


    AnalogPin *PWMpin;
    GPIO *dir;
    bool isInit;
};



