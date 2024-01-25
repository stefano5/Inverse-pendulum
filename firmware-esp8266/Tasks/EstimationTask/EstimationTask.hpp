#pragma once

#include "../../ErrorsHandle/ProgrammingErrors.hpp"
#include "../../GlobalDependacies.hpp"
#include "../../Scheduler/Task.hpp"
#include "../../Serial/DebugPrint.hpp"
#include "../../Hardware/Sensors/SensorsHandler.cpp"

class EstimationTask : public Task, public, public SensorsHandler {
public:	

    enum class Message_t : uint8_t {
        GET_ANGLES_POINTER
    };

    struct Values {
        float x_angle;
        float y_angle;
        float z_angle;
    };


    EstimationTask(int pin, const char* name, uint8_t _taskId, Period_t period, Priority_t priority) : 
    Task(name, _taskId, period, priority), DebugPrint(getTaskName()), taskId(_taskId) {}

	void initialize() override;
	void run() override;
    void handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) override;

private:
    uint32_t count=0;
    uint8_t taskId;
    Values angles;
};


