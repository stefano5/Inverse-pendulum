#ifndef __FAULT_MANAGER_HPP_
#define __FAULT_MANAGER_HPP_

#include "../ErrorsHandle/ProgrammingErrors.hpp"
#include <string.h>

class FaultsManager {
public:

/*
    enum class Sensors_t {
        HUMIDITY_1=0,
        HUMIDITY_2,
        HUMIDITY_3,
        HUMIDITY_4,
        TEMP_1,
        TEMP_2,
        HUMIDITY_AIR_1,
        HUMIDITY_AIR_2,
        LUX,
    };
*/
    enum class IdError_t : uint8_t {
        UNKNOWN_ERROR=0,
        HUMIDITY_1_BROKEN,
        HUMIDITY_2_BROKEN,
        HUMIDITY_3_BROKEN,
        HUMIDITY_4_BROKEN,
        TEMP_1_BROKEN,
        TEMP_2_BROKEN,
        HUMIDITY_AIR_1_BROKEN,
        HUMIDITY_AIR_2_BROKEN,
        LUX_BROKEN,

        CONTROL_ERROR,  // il controllo non riesce a partire perchè non c'è internet
        CONTROL_CANNOT_READ_MEASUREMENT, // c'è un sensore che stiamo provando a leggere, ma il sensore non risulta funzionante

        I2C_ERROR,   // nella scheda non sono presenti il numero di device i2c che dovrebbero esserci
        
        TOT
    };

    FaultsManager() {}

protected:

    class Fault {
    public:
        Fault();
        void make(IdError_t errorId, char* text, bool isFaultActivated);
        
        uint8_t getId() { return (uint8_t)errorId; }
        String toString() { return String(textFault); }


        bool isSet() { return activeFault; }
        void set() { activeFault = true; }
        void reset() { activeFault = false; }

        bool isActivated() { return isFaultActivated; }
        void active() { isFaultActivated = TRUE; }
        void disable() { isFaultActivated = FALSE; }
                
    private:
        IdError_t errorId;
        char textFault[20];
        bool activeFault;
        bool isFaultActivated;
    };

    bool isUniqueId(IdError_t idError);
    void createFault(IdError_t idError, char* textFault, bool isFaultActivated=TRUE);
    void setFault(IdError_t idError);
    void resetFault(IdError_t idError);    
    String getActivatedFaults();
    bool isFaultActive(IdError_t idError);

    inline uint8_t getFaultsNumber() { return counterFaults; }
    inline uint8_t getActivatedFaultsNumber() { return counterActivatedFault; }

    void registerFaults();

private:
    static Fault faults[(uint8_t)IdError_t::TOT];
    static uint8_t counterFaults;
    static uint8_t counterActivatedFault;
};


#endif