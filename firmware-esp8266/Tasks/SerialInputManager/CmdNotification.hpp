/*
 * CmdNotification.hpp
 *
 *  Created on: May 26, 2023
 *      Author: Maugeri
 */

#ifndef TASKS_SERIALINPUTMANAGER_CMDNOTIFICATION_HPP_
#define TASKS_SERIALINPUTMANAGER_CMDNOTIFICATION_HPP_

#include "../../ErrorsHandle/ProgrammingErrors.hpp"
#include "../../Scheduler/Task.hpp"

class CmdNotification {
public:
	static constexpr uint8_t MAX_REG_CMD = 20;	// Maximum number of commands that can be registered

	CmdNotification(Task* fatherPointer) : ref(fatherPointer) {}


	struct TaskRegistration {
		uint8_t idRegisteredTask;		// Here, I save the ID of the task that wants to receive a message when a registered command arrives
		char desiredCmd;		// This is the command that, if received from the serial, triggers a message to be sent to the task
		uint32_t mapCmd;		// This is the messageBody that the handleMessage() of the registered task will receive; the messageType will be (uint8_t)Messages_t::INCOMING_CMD_FROM_SERIAL
	} taskRegs[MAX_REG_CMD];
	uint8_t countRegisteredTask=0;

	void notifyTasks(const char cmd) {
		ASSERT_PTR(ref);	// The constructor must be called from a task, passing "this" as a parameter.

		for (uint8_t i=0; i<countRegisteredTask; ++i) {
			if (taskRegs[i].desiredCmd == cmd) {
				ref->sendMessage(taskRegs[i].idRegisteredTask, (uint8_t)GlobalMessages_t::INCOMING_CMD_FROM_SERIAL, taskRegs[i].mapCmd);
			}
		}
	}

private:
	CmdNotification() {} // do not use it

	Task* ref;
};




#endif /* TASKS_SERIALINPUTMANAGER_CMDNOTIFICATION_HPP_ */
