#include "../ErrorsHandle/ProgrammingErrors.hpp"
#include "Task.hpp"


uint8_t Task::uid=0u;
Task* Task::task[TOT_TASKS] = SET_ALL_ITEMS_TO_NULL; 
Scheduler* Task::schedulerPtr = nullptr;

Task::Task(const char* name, uint8_t taskId, Period_t period, Priority_t priority) {
    ASSERT_STR(name, LEN_NAME);          // Task name too long
    FATAL_ERROR_IF_TRUE(uid + 1 > TOT_TASKS);		// Reached the maximum limit of tasks. Update the TOT_TASKS macro in GlobalDependencies.hpp.

    strcpy(this->name, name);
    this->period = period;
    this->priority = priority;

    for (uint8_t i=0; i<uid; i++) {
    	FATAL_ERROR_IF_TRUE(taskId == task[i]->taskId); // The ID passed by the constructor must be unique.
    }

    this->taskId = taskId;
    task[uid++] = this;
    enableRunTask();	// By default, the run method of a task is active.
}

Task* Task::getPointerFromNameTask(NameTask id) {
	for (uint8_t i=0; i<uid; ++i) {
		if ((task[i] != nullptr) && (task[i]->getTaskId() == (uint8_t)id)) {
			return task[i];
		}
	}
	return nullptr;	// Requested a task that does not exist.
}


void Task::sendMessage(uint8_t idReceiver, uint8_t messageType, uint32_t messageBody) {
    ASSERT_PTR(schedulerPtr);   // Registration in Scheduler->setup() was not performed.
    ASSERT_PTR(Task::getPointerFromNameTask((NameTask)idReceiver));	// Trying to send a message to a task that does not exist.

    schedulerPtr->sendMessage(
            getTaskId(),
            idReceiver, 
            messageType, 
            messageBody
        ); 
}


void Task::handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) {
	NOT_USED(taskSender);
	NOT_USED(messageType);
	NOT_USED(messageBody);

	FATAL_ERROR;	// An application task received a message without having implemented the required function. This function is necessary to allow the task to receive messages.
}

const char* Task::getInfoTask() {
	for (uint16_t i=0; i<LEN_INFO_BUF; ++i) infoBuf[i] = 0;

	sprintf(infoBuf, "Task: [%s] has priority: %s and period: %d ms. The run() method %s being executed.",
			getTaskName(),
			NAME_PRIORITY(getPriority()),
			(uint16_t)getPeriod(),
			(isRunEnable() ? "":"NON ")
	);

	return infoBuf;
}


void Task::requestMessageIfCmdIsReceived(char cmd, uint32_t mapCmd) {
	Task* serialManagerTask = getPointerFromNameTask(NameTask::SerialManager);
	if (serialManagerTask != nullptr) {
		ASSERT_PTR(serialManagerTask);

		serialManagerTask->registerTask(getTaskId(), cmd, mapCmd);
	}
}

void Task::requestMessageIfCmdIsReceived(char cmd) {
	requestMessageIfCmdIsReceived(cmd, (uint32_t)cmd);
}



void Task::registerTask(uint8_t idRegTask, char cmd, uint32_t mapCmd) {
	NOT_USED(idRegTask);
	NOT_USED(cmd);
	NOT_USED(mapCmd);

	FATAL_ERROR; // Interface method, implemented only by SerialInputManager if present.
	// If SerialInputManager is not present, do not use this method.
}


