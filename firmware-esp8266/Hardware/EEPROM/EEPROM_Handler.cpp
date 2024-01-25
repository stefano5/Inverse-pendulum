#ifndef _ESP_EEPROM_HANDLER_CPP_
#define _ESP_EEPROM_HANDLER_CPP_


#include "EEPROM_Handler.hpp"

/*
EEPROM_Handler::Target target {
    {10, 80},         // ummidità dei vasi
    {   
        8, 30, 0,       // ora avvio irrigazione
        8, 30, 45       // ora fine irrigazione
    }, {
        40, 8, 21       // soglia a 40%, range orario dalle 8 alle 21 (si accende alle 8 e si spegne alle 21)
    }, {
        85, 15, 18      // soglia di 85% per l'umidità, di 15 gradi come soglia minima e 18 come soglia massima
    }
};
*/
EEPROM_Handler::Target EEPROM_Handler::target {
    {10, 80},         // ummidità dei vasi
    {   
        8, 30, 0,       // ora avvio irrigazione
        8, 30, 45       // ora fine irrigazione
    }, {
        40, 8, 21       // soglia a 40%, range orario dalle 8 alle 21 (si accende alle 8 e si spegne alle 21)
    }, {
        85, 15, 18      // soglia di 85% per l'umidità, di 15 gradi come soglia minima e 18 come soglia massima
    },
    {   // control settings
        0
    }
};


void EEPROM_Handler::initEEPROM() {
    uint8_t address=0;
    EEPROM.begin(this->size);

    EEPROM.get(address++, target.humidityVasesAM.humidityVasesMinTh);
    EEPROM.get(address++, target.humidityVasesAM.humidityVasesMaxTh);

    EEPROM.get(address++, target.pumpsHM.hhStart);
    EEPROM.get(address++, target.pumpsHM.mmStart);
    EEPROM.get(address++, target.pumpsHM.ssStart);
    EEPROM.get(address++, target.pumpsHM.hhEnd);
    EEPROM.get(address++, target.pumpsHM.mmEnd);
    EEPROM.get(address++, target.pumpsHM.ssEnd);
    
    EEPROM.get(address++, target.brightness.brightnessTh);
    EEPROM.get(address++, target.brightness.hhStart);
    EEPROM.get(address++, target.brightness.hhEnd);
    
    EEPROM.get(address++, target.tempAndHum.humidityAirTh);
    EEPROM.get(address++, target.tempAndHum.temperatureMinTh);
    EEPROM.get(address++, target.tempAndHum.temperatureMaxTh);

    EEPROM.get(address++, target.controlSettings.selectorControl);
}

void EEPROM_Handler::resetEEPROM() {
    target.humidityVasesAM.humidityVasesMinTh = 10;
    target.humidityVasesAM.humidityVasesMaxTh = 80;

    target.pumpsHM.hhStart = 8; 
    target.pumpsHM.mmStart = 30; 
    target.pumpsHM.ssStart = 0; 
    target.pumpsHM.hhEnd = 8;
    target.pumpsHM.mmEnd = 30;
    target.pumpsHM.ssEnd = 45;
    target.brightness.brightnessTh = 40;
    target.brightness.hhStart = 8;
    target.brightness.hhEnd = 21; 
    
    target.tempAndHum.humidityAirTh = 85; 
    target.tempAndHum.temperatureMinTh = 15;
    target.tempAndHum.temperatureMaxTh = 18; 

    target.controlSettings.selectorControl = 0;

    uint8_t address=0;
    EEPROM.put(address++, target.humidityVasesAM.humidityVasesMinTh);
    EEPROM.put(address++, target.humidityVasesAM.humidityVasesMaxTh);

    EEPROM.put(address++, target.pumpsHM.hhStart);
    EEPROM.put(address++, target.pumpsHM.mmStart); 
    EEPROM.put(address++, target.pumpsHM.ssStart); 
    EEPROM.put(address++, target.pumpsHM.hhEnd);
    EEPROM.put(address++, target.pumpsHM.mmEnd);
    EEPROM.put(address++, target.pumpsHM.ssEnd);
    EEPROM.put(address++, target.brightness.brightnessTh);
    EEPROM.put(address++, target.brightness.hhStart);
    EEPROM.put(address++, target.brightness.hhEnd); 

    EEPROM.put(address++, target.tempAndHum.humidityAirTh); 
    EEPROM.put(address++, target.tempAndHum.temperatureMinTh);
    EEPROM.put(address++, target.tempAndHum.temperatureMaxTh);
    EEPROM.put(address++, target.controlSettings.selectorControl);

    FATAL_ERROR_IF_FALSE(EEPROM.commit());
}

uint8_t EEPROM_Handler::Target::getValueById(uint8_t id) {
    switch (id) {
        case 0: return humidityVasesAM.humidityVasesMinTh;
        case 1: return humidityVasesAM.humidityVasesMaxTh;
        case 2: return pumpsHM.hhStart;
        case 3: return pumpsHM.mmStart;
        case 4: return pumpsHM.ssStart;
        case 5: return pumpsHM.hhEnd;
        case 6: return pumpsHM.mmEnd;
        case 7: return pumpsHM.ssEnd;
        case 8: return brightness.brightnessTh;
        case 9: return brightness.hhStart;
        case 10: return brightness.hhEnd;
        case 11: return tempAndHum.humidityAirTh;
        case 12: return tempAndHum.temperatureMinTh;
        case 13: return tempAndHum.temperatureMaxTh;
        case 14: return controlSettings.selectorControl;
        default: FATAL_ERROR; return 0;
    }
}


void EEPROM_Handler::Target::increaseValueById(uint8_t id) {
    switch (id) {
        case 0:
            humidityVasesAM.humidityVasesMinTh = (humidityVasesAM.humidityVasesMinTh + 1) % 100;
            setTargetHumidityVaseMinTh(humidityVasesAM.humidityVasesMinTh);
            break;
        case 1:
            humidityVasesAM.humidityVasesMaxTh = (humidityVasesAM.humidityVasesMaxTh + 1) % 100;
            setTargetHumidityVaseMaxTh(humidityVasesAM.humidityVasesMaxTh);
            break;
        case 2:
            pumpsHM.hhStart = (pumpsHM.hhStart + 1) % 24;
            setStartPumpsHh(pumpsHM.hhStart);
            break;
        case 3:
            pumpsHM.mmStart = (pumpsHM.mmStart + 1) % 60;
            setStartPumpsMm(pumpsHM.mmStart);
            break;
        case 4:
            pumpsHM.ssStart = (pumpsHM.ssStart + 1) % 60;
            setStartPumpsSs(pumpsHM.ssStart);
            break;
        case 5:
            pumpsHM.hhEnd = (pumpsHM.hhEnd + 1) % 60;
            setEndPumpsHh(pumpsHM.hhEnd);
            break;
        case 6:
            pumpsHM.mmEnd = (pumpsHM.mmEnd + 1) % 60;
            setEndPumpsMm(pumpsHM.mmEnd);
            break;
        case 7:
            pumpsHM.ssEnd = (pumpsHM.ssEnd + 1) % 60;
            setEndPumpsSs(pumpsHM.ssEnd);
            break;
        case 8:
            brightness.brightnessTh = (brightness.brightnessTh + 1) % 100;
            setBrightnessTh(brightness.brightnessTh);
            break;
        case 9: 
            brightness.hhStart = (brightness.hhStart + 1) % 24;
            setBrightnessHhStart(brightness.hhStart);
            break;
        case 10:
            brightness.hhEnd = (brightness.hhEnd + 1) % 24;
            setBrightnessHhEnd(brightness.hhEnd);
            break;
        case 11: 
            tempAndHum.humidityAirTh = (tempAndHum.humidityAirTh + 1) % 100;
            setTempAndHum_humidityTh(tempAndHum.humidityAirTh);
            break;
        case 12: 
            tempAndHum.temperatureMinTh = (tempAndHum.temperatureMinTh + 1) % 50;
            setTempAndHum_tempMinTh(tempAndHum.temperatureMinTh);
            break;
        case 13: 
            tempAndHum.temperatureMaxTh = (tempAndHum.temperatureMaxTh + 1) % 50;
            setTempAndHum_tempMaxTh(tempAndHum.temperatureMaxTh);
            break;
        case 14:
            controlSettings.selectorControl = (controlSettings.selectorControl + 1) % 2;
            setSelecterControl(controlSettings.selectorControl);
            break;
        default: FATAL_ERROR;
    }
}

void EEPROM_Handler::Target::decreaseValueById(uint8_t id) {
    switch (id) {
        case 0:
            humidityVasesAM.humidityVasesMinTh = (humidityVasesAM.humidityVasesMinTh) == 0 ? 100 : (humidityVasesAM.humidityVasesMinTh - 1);
            setTargetHumidityVaseMinTh(humidityVasesAM.humidityVasesMinTh);
            break;
        case 1:
            humidityVasesAM.humidityVasesMaxTh = (humidityVasesAM.humidityVasesMaxTh) == 0 ? 100 : (humidityVasesAM.humidityVasesMaxTh - 1);
            setTargetHumidityVaseMaxTh(humidityVasesAM.humidityVasesMaxTh);
            break;
        case 2:
            pumpsHM.hhStart = (pumpsHM.hhStart) == 0 ? 23 : (pumpsHM.hhStart - 1);
            setStartPumpsHh(pumpsHM.hhStart);
            break;
        case 3:
            pumpsHM.mmStart = (pumpsHM.mmStart) == 0 ? 59 : (pumpsHM.mmStart - 1);
            setStartPumpsMm(pumpsHM.mmStart);
            break;
        case 4:
            pumpsHM.ssStart = (pumpsHM.ssStart) == 0 ? 59 : (pumpsHM.ssStart - 1);
            setStartPumpsSs(pumpsHM.ssStart);
            break;
        case 5:
            pumpsHM.hhEnd = (pumpsHM.hhEnd) == 0 ? 23 : (pumpsHM.hhEnd - 1);
            setEndPumpsHh(pumpsHM.hhEnd);
            break;
        case 6:
            pumpsHM.mmEnd = (pumpsHM.mmEnd) == 0 ? 59 : (pumpsHM.mmEnd - 1);
            setEndPumpsMm(pumpsHM.mmEnd);
            break;
        case 7:
            pumpsHM.ssEnd = (pumpsHM.ssEnd) == 0 ? 59 : (pumpsHM.ssEnd - 1);
            setEndPumpsSs(pumpsHM.ssEnd);
            break;
        case 8:
            brightness.brightnessTh = (brightness.brightnessTh) == 0 ? 100 : (brightness.brightnessTh - 1);
            setBrightnessTh(brightness.brightnessTh);
            break;
        case 9: 
            brightness.hhStart = (brightness.hhStart) == 0 ? 23 : (brightness.hhStart - 1);
            setBrightnessHhStart(brightness.hhStart);
            break;
        case 10:
            brightness.hhEnd = (brightness.hhEnd) == 0 ? 23 : (brightness.hhEnd - 1);
            setBrightnessHhEnd(brightness.hhEnd);
            break;
        case 11: 
            tempAndHum.humidityAirTh = (tempAndHum.humidityAirTh) == 0 ? 100 : (tempAndHum.humidityAirTh - 1); 
            setTempAndHum_humidityTh(tempAndHum.humidityAirTh);
            break;
        case 12:
            tempAndHum.temperatureMinTh = (tempAndHum.temperatureMinTh) == 0 ? 50 : (tempAndHum.temperatureMinTh - 1);
            setTempAndHum_tempMinTh(tempAndHum.temperatureMinTh);
            break;
        case 13:
            tempAndHum.temperatureMaxTh = (tempAndHum.temperatureMaxTh) == 0 ? 50 : (tempAndHum.temperatureMaxTh - 1);
            setTempAndHum_tempMaxTh(tempAndHum.temperatureMaxTh);
            break;
        case 14:
            controlSettings.selectorControl = (controlSettings.selectorControl == 0) ? 1 : 0;
            setSelecterControl(controlSettings.selectorControl);
            break;
        default: FATAL_ERROR;
    }    
}



#endif