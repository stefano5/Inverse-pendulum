/*
 * FWSerial.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: Maugeri
 */

#include "FWSerial.hpp"


HardwareSerial* FWSerial::handle = nullptr;

FWSerial::FWSerial(void* _owner) : owner(this) {}
bool FWSerial::isEnabled = true;


void FWSerial::print(const char* str) {
	//ASSERT_PTR(handle);	// non è stato dato l'handle della seriale
    if (handle == nullptr) handle = &Serial;
	// probabilmente si è messa una stampa in un costruttore
	if (owner != this) {
		if (!isEnabled) return;
	}

    handle->print((const char*)str);
}


void FWSerial::clearTerminal() {
	print(clear_screen);
	print(move_to_home);
}

void FWSerial::setBoldFont() {
	print(bold);
}

void FWSerial::setFaintFont() {
	print(faint);
}

void FWSerial::resetAttributes() {
	print(reset_att);
}


uint16_t FWSerial::itoa(uint32_t value, char *ptr) {
    uint16_t count = 0u;
    uint32_t temp = 0u;

    ASSERT_PTR(ptr);	/// parametro in ingresso null
    //if (ptr == nullptr) FATAL_ERROR;

    if(value == 0) {
        *ptr = '0';
        return 1;
    }

    if(value < 0) {
        value *= (-1);
        *ptr++ = '-';
        count++;
    }

    for (temp = value; temp > 0; temp/=10, ptr++);
    *ptr = '\0';

    for (temp=value; temp>0; temp/=10) {
        *--ptr = ((uint8_t)(temp%10)) + '0';
        count++;
    }
    return count;
}




