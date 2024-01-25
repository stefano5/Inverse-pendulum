/*
 * DebugPrint.cpp
 *
 *  Created on: Apr 19, 2023
 *      Author: Maugeri
 */

#include "DebugPrint.hpp"

bool DebugPrint::isFirstUsage = true;


FWSerial DebugPrint::outStream() {

 	if (isFirstUsage) {
		isFirstUsage = false;
		*this << clear_screen << move_to_home;
	}
	*this << "[" << set_foreground(cyan) << nameTask << reset_att << "] ";
	return *this;
}


