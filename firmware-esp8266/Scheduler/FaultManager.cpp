#ifndef __FAULT_MANAGER_CPP_
#define __FAULT_MANAGER_CPP_

#include "FaultManager.hpp"

FaultsManager::Fault FaultsManager::faults[(uint8_t)FaultsManager::IdError_t::TOT];
uint8_t FaultsManager::counterFaults=0;
uint8_t FaultsManager::counterActivatedFault=0;



FaultsManager::Fault::Fault() {
    for(uint8_t i=0; i<20; ++i) 
    textFault[i] = 0; 
    activeFault = false;    // i fault partono da non settati
    isFaultActivated = true;
    errorId = IdError_t::UNKNOWN_ERROR; 
}

void FaultsManager::Fault::make(IdError_t errorId, char* text, bool isFaultActivated) { 
    this->errorId = errorId;
    this->isFaultActivated = isFaultActivated;
    strcpy(textFault, text); 
}

void FaultsManager::registerFaults() {
    createFault(FaultsManager::IdError_t::HUMIDITY_1_BROKEN, (char*)"Humidity_1 fault");
    createFault(FaultsManager::IdError_t::HUMIDITY_2_BROKEN, (char*)"Humidity_2 fault");
    createFault(FaultsManager::IdError_t::HUMIDITY_3_BROKEN, (char*)"Humidity_3 fault", FALSE);  // dichiaro questo fault, ma lo disabilito perchè nel sistema questi due sensori non servono
    createFault(FaultsManager::IdError_t::HUMIDITY_4_BROKEN, (char*)"Humidity_4 fault", FALSE);
    createFault(FaultsManager::IdError_t::TEMP_1_BROKEN, (char*)"Temp up fault");
    createFault(FaultsManager::IdError_t::TEMP_2_BROKEN, (char*)"Temp down fault");
    createFault(FaultsManager::IdError_t::HUMIDITY_AIR_1_BROKEN, (char*)"Air humid. 1 fault");
    createFault(FaultsManager::IdError_t::HUMIDITY_AIR_2_BROKEN, (char*)"Air humid. 2 fault");
    createFault(FaultsManager::IdError_t::LUX_BROKEN, (char*)"Brightness fault");
    createFault(FaultsManager::IdError_t::CONTROL_ERROR, (char*)"Control fault");
    createFault(FaultsManager::IdError_t::CONTROL_CANNOT_READ_MEASUREMENT, (char*)"Control & meas");
    createFault(FaultsManager::IdError_t::I2C_ERROR, (char*)"I2C lost device");



    FATAL_ERROR_IF_TRUE(counterFaults != (uint8_t)FaultsManager::IdError_t::TOT - 1);   // si è aggiunto un IdError_t ma non lo si è registrato qui
}

bool FaultsManager::isUniqueId(IdError_t idError) {
    for (uint8_t i=0; i<getFaultsNumber(); i++) {
        if (faults[i].getId() == (uint8_t)idError)
            return false;
    }
    return true;
}

void FaultsManager::createFault(IdError_t idError, char* textFault, bool isFaultActivated) {
    FATAL_ERROR_IF_FALSE(isUniqueId(idError));

    FATAL_ERROR_IF_FALSE(counterFaults < (uint8_t)FaultsManager::IdError_t::TOT);  // leggilo come un for(.., counterFaults < FaultsManager::IdError_t::TOT; ++)

    ASSERT_STR(textFault, 20);
    
    faults[counterFaults++].make(idError, textFault, isFaultActivated);
}

void FaultsManager::setFault(IdError_t idError) {
    for (uint8_t i=0; i<getFaultsNumber(); i++) {
        if (faults[i].getId() == (uint8_t)idError && faults[i].isActivated() && !faults[i].isSet()) {
            faults[i].set();
            counterActivatedFault++;
        }
    }
}

void FaultsManager::resetFault(IdError_t idError) {
    for (uint8_t i=0; i<getFaultsNumber(); i++) {
        if (faults[i].getId() == (uint8_t)idError && faults[i].isSet()) {
            faults[i].reset();
            counterActivatedFault--;
        } 
    }
}

String FaultsManager::getActivatedFaults() {
    String ret = "";
    for (uint8_t i=0; i<getFaultsNumber(); i++) {
        if (faults[i].isSet())
            ret += faults[i].toString() + "\n";
    }
    return ret;
}

bool FaultsManager::isFaultActive(IdError_t idError) {
    for (uint8_t i=0; i<getFaultsNumber(); i++) {
        if (faults[i].getId() == (uint8_t)idError) {
            return faults[i].isSet();
        }
    }
    FATAL_ERROR;    // non raggiungibile
    return false;
}


#endif
