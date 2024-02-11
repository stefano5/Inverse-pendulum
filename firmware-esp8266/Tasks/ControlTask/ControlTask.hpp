#pragma once

#include "../../ErrorsHandle/ProgrammingErrors.hpp"
#include "../../GlobalDependacies.hpp"
#include "../../Scheduler/Task.hpp"
#include "../../Serial/DebugPrint.hpp"
#include "../EstimationTask/EstimationTask.cpp"
#include "../../Hardware/Controller/PIDController.cpp"


class ControlTask : public Task, public DebugPrint {
public:	

    enum class Message_t {
        UPDATE_PID_ANGLES=7,
        UPDATE_PID_SPEED,
        TOGGLE_DEBUG_PRINT
    };

    ControlTask(HBridge *motor_l, HBridge *motor_r, const char* name, uint8_t _taskId, Period_t period, Priority_t priority) : 
        Task(name, _taskId, period, priority), DebugPrint(getTaskName()), angles(nullptr), motor_left(motor_l), motor_right(motor_r), debugPrint(false) {}

	void initialize() override;
	void run() override;
    void handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) override;

private:

    void updatePidGains(uint32_t gains, uint8_t whichPid);


    EstimationTask::Values *angles;
    PIDController controller_left;
    PIDController controller_right;
    PIDController controller_left_speed;
    PIDController controller_right_speed;
    
    HBridge *motor_left;
    HBridge *motor_right;
    bool debugPrint;
};


