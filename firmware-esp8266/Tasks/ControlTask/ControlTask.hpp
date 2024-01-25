#pragma once

#include "../../ErrorsHandle/ProgrammingErrors.hpp"
#include "../../GlobalDependacies.hpp"
#include "../../Scheduler/Task.hpp"
#include "../../Serial/DebugPrint.hpp"
#include "../EstimationTask/EstimationTask.cpp"


class ControlTask : public Task, public DebugPrint  {
public:	
    ControlTask(int pin, const char* name, uint8_t _taskId, Period_t period, Priority_t priority) : Task(name, _taskId, period, priority), DebugPrint(getTaskName()), taskId(_taskId) {}

	void initialize() override;
	void run() override;
    void handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) override;

private:
    EstimationTask::Values *angles;
    
};


