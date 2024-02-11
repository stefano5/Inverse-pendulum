/*
 * SerialInputManager.hpp
 *
 *  Created on: May 25, 2023
 *      Author: Maugeri
 */

#ifndef TASKS_SERIALINPUTMANAGER_SERIALINPUTMANAGER_HPP_
#define TASKS_SERIALINPUTMANAGER_SERIALINPUTMANAGER_HPP_

#include "../../ErrorsHandle/ProgrammingErrors.hpp"
#include "../../Scheduler/Task.hpp"
#include "../../GlobalDependacies.hpp"

#include "../../Serial/DebugPrint.hpp"
#include "../ControlTask/ControlTask.cpp"


#include "CmdNotification.hpp"

String acceptedCommands[] = {
	"inl",		// 0
	"decl",		// 1
	"inr",		// 2
	"decr",		// 3
	"for",		// 4
	"back",		// 5
	"reg1",		// 6
	"reg2",		// 7
	"printU",	// 8
	"printEst"	// 9
};

class SerialInputManager : public Task, public DebugPrint, public CmdNotification {
public:
	SerialInputManager(const char* name, uint8_t _taskId, Period_t period, Priority_t priority) : Task(name, _taskId, period, priority),
				DebugPrint(getTaskName(), this), CmdNotification(this) {}

	void initialize() override;
	void run() override;
    void handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) override;

protected:
	
	void checkStringAndUpdatePIDGains(const char* str, uint8_t calledFrom);

	/**
	 * @brief A task that wants to receive a specific command (i.e., a certain character) from the serial, calls this method.
	 *        It requests that the handleMessage() method be called if the user presses that character on the serial.
	 *        In this case, the registered task will receive a message of type: Messages_t::INCOMING_CMD_FROM_SERIAL
	 *        and as messageBody, it will receive uniqueIdMsg.
	 *
	 * */
	void registerTask(uint8_t idRegTask, char cmd, uint32_t mapCmd) override {
		FATAL_ERROR_IF_TRUE(countRegisteredTask >= MAX_REG_CMD); // ho registrato troppi messaggi, incrementare CmdNotification::MAX_REG_CMD

		taskRegs[countRegisteredTask].idRegisteredTask = idRegTask;
		taskRegs[countRegisteredTask].desiredCmd = cmd;
		taskRegs[countRegisteredTask].mapCmd = mapCmd;
		countRegisteredTask++;
	}

private:

	int16_t getIndex(String key);	

	/*
	 * @brief	get the handle for serial printing, assuming that regular API-based prints are disabled during the process.
	 * */
	inline FWSerial priorityOut() { return *this; }
};


#endif /* TASKS_SERIALINPUTMANAGER_SERIALINPUTMANAGER_HPP_ */
