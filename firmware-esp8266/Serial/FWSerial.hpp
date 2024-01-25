/*
 * FWSerial.hpp
 *
 *  Created on: Apr 17, 2023
 *      Author: Maugeri
 */

#ifndef SERIAL_SERIAL_HPP_
#define SERIAL_SERIAL_HPP_

#include "../ErrorsHandle/ProgrammingErrors.hpp"
#include <string.h>

class FWSerial {
public:
	FWSerial(void* _owner=nullptr);


    FWSerial& operator << (const char* s) {
        print(s);
        return *this;
    }

    FWSerial& operator <<(const uint8_t &s) {
        char temp[4];
        itoa(s, temp);
        print(temp);
        return *this;
    }

    FWSerial& operator <<(const uint16_t &s) {
        char temp[6];
        itoa(s, temp);
        print(temp);
        return *this;
    }

    FWSerial& operator <<(const uint32_t &s) {
        char temp[11];
        itoa(s, temp);
        print(temp);
        return *this;
    }

    FWSerial& operator <<(String s) {
        char *buf = (char*)malloc(s.length()+1);
        ASSERT_PTR(buf);
        for (uint8_t i=0; i<s.length()+1; ++i) buf[i] = '\0';
        s.toCharArray(buf, s.length()+1);

        print(buf);

        free(buf);
        return *this;
    }

    FWSerial& operator <<(const int8_t &s) {
        char temp[4];
        sprintf(temp, "%d", s);
        print(temp);
        return *this;
    }

    FWSerial& operator <<(const int16_t &s) {
        char temp[6];
        sprintf(temp, "%d", s);
        print(temp);
        return *this;
    }

    FWSerial& operator <<(const int32_t &s) {
        char temp[11];
        sprintf(temp, "%d", s);
        print(temp);
        return *this;
    }


    static void setHandle(HardwareSerial* usartHandle) {
        handle = usartHandle;
        handle->begin(115200);
        handle->println("\n\n");
    }

    bool isThereIncomingData() {
        return handle->available() > 0;
    }

    String getIncomingData() {
        return handle->readString();
    }

	void clearTerminal();
	void setBoldFont();
	void setFaintFont();
	void resetAttributes();
protected:
	void enableOutStream() { isEnabled = true; }
	void disableOutStream() { isEnabled = false; }

private:

	inline void print(const char* str);

	uint16_t itoa(uint32_t value, char *ptr);
    static HardwareSerial* handle;
    void* owner;
    static bool isEnabled;

};



#endif /* SERIAL_SERIAL_HPP_ */
