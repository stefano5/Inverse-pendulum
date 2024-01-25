/*
 * DebugPrint.hpp
 *
 *  Created on: Apr 19, 2023
 *      Author: Maugeri
 */

#ifndef SERIAL_DEBUGPRINT_HPP_
#define SERIAL_DEBUGPRINT_HPP_

#include "FWSerial.hpp"
#include "TerminalManipulator.hpp"

//
// | putty|xterm|serial reader
// |-------------------------|  -----> ASSE X
// |*(x=0,y=0)               |
// |                         |
// |                         |
// |                         |
// |                         |
// |                         |
// |_________________________|
// |
// |
// V ASSE y
//


class DebugPrint : public FWSerial {
public:
	DebugPrint(const char* nameOwnerTask, void* owner=nullptr) : FWSerial(owner), nameTask(nameOwnerTask) {}

	FWSerial outStream();

private:
	const char* nameTask;
    static bool isFirstUsage;
};



#endif /* SERIAL_DEBUGPRINT_HPP_ */
