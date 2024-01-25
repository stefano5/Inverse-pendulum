/*
 * SerialInputManager.cpp
 *
 *  Created on: May 26, 2023
 *      Author: Maugeri
 */

#include "SerialInputManager.hpp"

#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

void SerialInputManager::initialize() {
    outStream() << getInfoTask() << endl;
}

int16_t SerialInputManager::getIndex(String key) {
	for (uint8_t i=0; i < NUMITEMS(acceptedCommands); i++) {
		acceptedCommands[i].toLowerCase();
		//outStream() << "acceptedCommands[i]: " << acceptedCommands[i] << endl;
		if (key == acceptedCommands[i]) return i;
	}
	return -1;
}

void SerialInputManager::run() {
	if (isThereIncomingData()) {
		String incomingData = getIncomingData(); 	//read until timeout
		incomingData.trim();
		//outStream() << "ricevo: [" << incomingData << "]" << endl;
		String key = incomingData.substring(0, incomingData.indexOf('@'));
		key.toLowerCase();
		incomingData = incomingData.substring(incomingData.indexOf('@') + 1);
		outStream() << "key: [" << key << "]" << endl;

		switch (getIndex(key)) {
		case 0:	// example

			break;
		default:
			outStream() << ": [" << key << "] not found as key. Available commands are:" << endl;
			for (uint8_t i=0; i < NUMITEMS(acceptedCommands); i++) {
				outStream() << acceptedCommands[i] << endl;
			}
			outStream() << endl;
			break;
		}
	}
}

void SerialInputManager::handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) {
	FATAL_ERROR;
}
