#ifndef HARDWARE_GPIO_GPIO_HPP_
#define HARDWARE_GPIO_GPIO_HPP_


#include "../../ErrorsHandle/ProgrammingErrors.hpp"

class GPIO {
public:
	enum class IOMode : uint8_t {
		INPUT_MODE=0,
		OUTPUT_MODE,
		EXTERN,
		CUSTOM
	};

	GPIO(uint8_t Pin, IOMode pinMode, bool directLogic_=true);
	GPIO(IOMode pinMode, bool directLogic_=true) : mode(pinMode), directLogic(directLogic_), wasEn(false) {}

	virtual void initGPIO();
	virtual void turnOn();
	virtual void turnOff();
	virtual void toggle();

	/*
	 * @return true se il pin è on, false altrimenti
	*/
	virtual bool readValue();

	inline IOMode getMode() {return mode;}
	inline bool isDirectLogic() {return directLogic;}
	inline void setMode(IOMode newMode) {mode = newMode;}
	inline void setDirectLogic(uint8_t directLogic) {this->directLogic = directLogic;}

	String toString() { return "Pin: (" + String(pin) + "). mode: (" + String((uint8_t)getMode()) + ")";}

protected:
	uint8_t pin;
	IOMode mode;
	bool directLogic;
	bool wasEn;
};



#endif /* HARDWARE_GPIO_GPIO_HPP_ */
