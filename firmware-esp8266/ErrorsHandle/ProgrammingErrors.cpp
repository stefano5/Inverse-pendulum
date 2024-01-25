/*
 * ProgrammingErrors.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: Maugeri
 */
#include "ProgrammingErrors.hpp"


inline void __fastPrintStr(const char* str) {
    //Serial.print((const char*)str);
    // send to serial
}

inline uint16_t __itoa(uint32_t value, char *ptr) {
    uint16_t count = 0u;
    uint32_t temp = 0u;

    ASSERT_PTR(ptr);

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


inline void __fastPrintInt(uint32_t val) {
	char temp[11];
	__itoa(val, temp);
	__fastPrintStr(temp);
}

inline void hardResetBoard() {
    FATAL_ERROR; // to be implemented
}



void fatalError(const char* errorType, const char* funcName, const char* fileName, uint32_t lineNumber) {
    // re-initialize serial comunication
    // insert here 

    // print error with informations
    __fastPrintStr("\n[");
    __fastPrintStr(set_foreground(red));
    __fastPrintStr(bold);
    __fastPrintStr(errorType == nullptr ? "Fatal error" : errorType);
    __fastPrintStr(reset_att);
    __fastPrintStr("]");

    if (funcName != nullptr) {
        __fastPrintStr(" on ");
        __fastPrintStr(funcName);
    }

    if (fileName != nullptr) {
        __fastPrintStr(" at ");
        __fastPrintStr(fileName);
        __fastPrintStr(" : ");
        __fastPrintInt(lineNumber);
    }

    __fastPrintStr("\n");

    while(1) {
		delay(1000);
	}
}

