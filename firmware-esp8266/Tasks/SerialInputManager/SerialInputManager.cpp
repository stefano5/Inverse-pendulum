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
	static int vall = 0;
	static int valr = 0;
	static int reg1ToBeUpdated=false;
	static int reg2ToBeUpdated=false;
	
	if (isThereIncomingData()) {
		String incomingData = getIncomingData(); 	//read until timeout
		incomingData.trim();
		//outStream() << "ricevo: [" << incomingData << "]" << endl;
		String key = incomingData.substring(0, incomingData.indexOf('@'));
		key.toLowerCase();
		incomingData = incomingData.substring(incomingData.indexOf('@') + 1);
		outStream() << "key: [" << key << "]" << endl;

		switch (getIndex(key)) {
		case 0: vall += 10;
			break;
		case 1:vall -= 10;
			break;
		case 2: valr += 10;
			break;
		case 3: valr -= 10;
			break;
		case 4:
		case 5:
			break;
		case 6: //reg1, update reg1
			reg1ToBeUpdated = true;
			break;
		case 7: //reg2, update reg2
			reg2ToBeUpdated = true;
			break;
		case 8:
			sendMessage((uint8_t)NameTask::Control, (uint32_t)ControlTask::Message_t::TOGGLE_DEBUG_PRINT);
			break;
		case 9:
			sendMessage((uint8_t)NameTask::Estimation, (uint32_t)EstimationTask::Message_t::TOGGLE_DEBUG_PRINT);
			break;
		default:
			if (reg1ToBeUpdated) {
				reg1ToBeUpdated = false;
    			//sendMessage((uint8_t)NameTask::Estimation, (uint32_t)EstimationTask::Message_t::GET_ANGLES_POINTER);
				checkStringAndUpdatePIDGains(key.c_str(), 1);
				break;
			} else if (reg2ToBeUpdated) {
				reg2ToBeUpdated = false;
				checkStringAndUpdatePIDGains(key.c_str(), 2);
				break;
			}

			outStream() << ": [" << key << "] not found as key. Available commands are:" << endl;
			for (uint8_t i=0; i < NUMITEMS(acceptedCommands); i++) {
				outStream() << acceptedCommands[i] << endl;
			}
			outStream() << endl;
			break;
		}
	}
}

void SerialInputManager::checkStringAndUpdatePIDGains(const char* str, uint8_t calledFrom) {
	uint8_t newKp;
	uint8_t newKi;
	uint8_t newKd;

    uint8_t hashCount = 0;
    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] == '#') {
            hashCount++;
        }
    }

	if (hashCount != 2) {
		outStream() << "Input error. Format: kp#ki#kd" << endl;
	}
	
    sscanf(str, "%hhu#%hhu#%hhu", &newKp, &newKi, &newKd);

	uint32_t message=0;
	message = newKp;
	message = (message<<8) | newKi;
	message = (message<<8) | newKd;

	if (calledFrom == 1) {
		sendMessage((uint8_t)NameTask::Control, (uint32_t)ControlTask::Message_t::UPDATE_PID_ANGLES, message);
	} else if (calledFrom == 2) {
		sendMessage((uint8_t)NameTask::Control, (uint32_t)ControlTask::Message_t::UPDATE_PID_SPEED, message);
	} else {
		FATAL_ERROR;	// input error
	}
}

void SerialInputManager::handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) {
	outStream() << "Task sender: [" << taskSender << "]; message type: " <<  messageType << endl;
	FATAL_ERROR;
}

