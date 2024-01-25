#ifndef _ESP_EEPROM_HANDLER_HPP_
#define _ESP_EEPROM_HANDLER_HPP_

#include <ESP_EEPROM.h>




class EEPROM_Handler {
public:

    EEPROM_Handler(size_t size=16) {
        if (size < 16) this->size = 16;  // The minimum size is 16
        else this->size = size;
    }

    void initEEPROM();
    void resetEEPROM();


//protected:

    struct Target {
        struct {
            uint8_t humidityVasesMinTh;     // sotto questa soglia parte l'irrigazione
            uint8_t humidityVasesMaxTh;     // sopra questa soglia si ferma l'irrigazione, in mezzo non fa nulla
        } humidityVasesAM;       // target vasi se in modalità automatica
        struct {
            uint8_t hhStart, mmStart, ssStart;       // ora, minuto e secondo target in cui parte l'irrigazione
            uint8_t hhEnd, mmEnd, ssEnd;             // ora, minuto e secondo target in cui parte l'irrigazione
        } pumpsHM;               // target orari di irrigazione se in modalità ibrida
        struct {
            uint8_t brightnessTh;           // sotto questa soglia si accende la luce, sopra si spegne
            uint8_t hhStart;                // ora in cui può accendersi la luce
            uint8_t hhEnd;                  // ora in cui deve spegnersi la luce
        } brightness;           // target di luminosità se in modalità ibrida o automatica

        struct {
            uint8_t humidityAirTh;          // sotto questa soglia si passa al controllo di temperatura, sopra si accendono le ventole e si spegne il riscaldamento
            uint8_t temperatureMinTh;       // sotto questa soglia si accende il riscaldamento
            uint8_t temperatureMaxTh;       // sopra questa soglia si spegne il riscaldamento  
        } tempAndHum;

        struct {
            uint8_t selectorControl;        // 0: HUMIDITY_FIRST; 1: TEMPERATURE_FIRST
        } controlSettings;

        uint8_t getValueById(uint8_t id);
        void increaseValueById(uint8_t id);
        void decreaseValueById(uint8_t id);
        

        void setTargetHumidityVaseMinTh(uint8_t val) { EEPROM.put(0, val); humidityVasesAM.humidityVasesMinTh = val; EEPROM.commit();}
        void setTargetHumidityVaseMaxTh(uint8_t val) { EEPROM.put(1, val); humidityVasesAM.humidityVasesMaxTh = val; EEPROM.commit();}

        void setStartPumpsHh(uint8_t val) {EEPROM.put(2, val); pumpsHM.hhStart = val; EEPROM.commit();}
        void setStartPumpsMm(uint8_t val) {EEPROM.put(3, val); pumpsHM.mmStart = val; EEPROM.commit();}
        void setStartPumpsSs(uint8_t val) {EEPROM.put(4, val); pumpsHM.ssStart = val; EEPROM.commit();}
        void setEndPumpsHh(uint8_t val) {EEPROM.put(5, val); pumpsHM.hhEnd = val; EEPROM.commit();}
        void setEndPumpsMm(uint8_t val) {EEPROM.put(6, val); pumpsHM.mmEnd = val; EEPROM.commit();}
        void setEndPumpsSs(uint8_t val) {EEPROM.put(7, val); pumpsHM.ssEnd = val; EEPROM.commit();}

        void setBrightnessTh(uint8_t val) {EEPROM.put(8, val); brightness.brightnessTh = val; EEPROM.commit();}
        void setBrightnessHhStart(uint8_t val) {EEPROM.put(9, val); brightness.hhStart = val; EEPROM.commit();}
        void setBrightnessHhEnd(uint8_t val) {EEPROM.put(10, val); brightness.hhEnd = val; EEPROM.commit();}
        
        void setTempAndHum_humidityTh(uint8_t val) {EEPROM.put(11, val); tempAndHum.humidityAirTh = val; EEPROM.commit();}
        void setTempAndHum_tempMinTh(uint8_t val) {EEPROM.put(12, val); tempAndHum.temperatureMinTh = val; EEPROM.commit();}
        void setTempAndHum_tempMaxTh(uint8_t val) {EEPROM.put(13, val); tempAndHum.temperatureMaxTh = val; EEPROM.commit();}

        void setSelecterControl(uint8_t val) {EEPROM.put(14, val); controlSettings.selectorControl = val; EEPROM.commit();}
    };

    static Target target;



private:
    size_t size;
};

#endif




