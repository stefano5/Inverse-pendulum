#ifndef GPIO_CPP
#define GPIO_CPP
#include "GPIO.hpp"


GPIO::GPIO(uint8_t Pin_, IOMode mode_, bool directLogic_) : pin(Pin_), mode(mode_), directLogic(directLogic_), wasEn(FALSE) {}

void GPIO::initGPIO() {
	ASSERT_EQUAL(mode, IOMode::EXTERN);	// devo usare il metodo derivato

	wasEn = true;
	if (mode != IOMode::CUSTOM) {
		pinMode(pin, mode == IOMode::OUTPUT_MODE ? OUTPUT : INPUT);
	} else {
		FATAL_ERROR;
		// altre modalità da implementare
	}
}

void GPIO::turnOn() {
	ASSERT_EQUAL(mode, IOMode::EXTERN);	// devo usare il metodo derivato
	ASSERT(wasEn);	// PIN non inizializzato

	if (mode == IOMode::OUTPUT_MODE) {
		digitalWrite((uint8_t)pin, directLogic ? HIGH : LOW);
	} else {
		FATAL_ERROR; // non si può comandare un pin che non è in modalità output
	}
}

void GPIO::turnOff() {
	ASSERT_EQUAL(mode, IOMode::EXTERN);	// devo usare il metodo derivato
	ASSERT(wasEn);	// PIN non inizializzato

	if (mode == IOMode::OUTPUT_MODE) {
		digitalWrite((uint8_t)pin, directLogic ? LOW : HIGH);
	} else {
		FATAL_ERROR; // non si può comandare un pin che non è in modalità output
	}
}


void GPIO::toggle() {
	ASSERT_EQUAL(mode, IOMode::EXTERN);	// devo usare il metodo derivato
	ASSERT(wasEn);	// PIN non inizializzato

	if (mode == IOMode::OUTPUT_MODE) {
		if (readValue()) {
			turnOff();
		} else {
			turnOn();
		}
	} else {
		FATAL_ERROR; // non si può comandare un pin che non è in modalità output
	}
}

bool GPIO::readValue() {
	ASSERT_EQUAL(mode, IOMode::EXTERN);	// devo usare il metodo derivato

	return digitalRead((uint8_t)pin) == (directLogic ? HIGH : LOW); 
}



#endif