#pragma once

#include "../../ErrorsHandle/ProgrammingErrors.hpp"
#include "../../GlobalDependacies.hpp"
#include "../../Scheduler/Task.hpp"
#include "../../Serial/DebugPrint.hpp"
#include "../EstimationTask/EstimationTask.cpp"
#include "../../Hardware/Controller/PIDController.cpp"


class ControlTask : public Task, public DebugPrint {
public:	
    ControlTask(HBridge *motor_l, HBridge *motor_r, const char* name, uint8_t _taskId, Period_t period, Priority_t priority) : 
        Task(name, _taskId, period, priority), DebugPrint(getTaskName()), angles(nullptr), motor_left(motor_l), motor_right(motor_r) {}

	void initialize() override;
	void run() override;
    void handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) override;

private:
    EstimationTask::Values *angles;
    PIDController controller_left;
    PIDController controller_right;
    
    HBridge *motor_left;
    HBridge *motor_right;
    
};


