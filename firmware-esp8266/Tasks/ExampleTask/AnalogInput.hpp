#ifndef ANALOG_INPUT_HPP__
#define ANALOG_INPUT_HPP__

#include "../../ErrorsHandle/ProgrammingErrors.hpp"
#include "../../GlobalDependacies.hpp"
#include "../../Scheduler/Task.hpp"
#include "../../Serial/DebugPrint.hpp"

class AnalogInput : public Task, public DebugPrint {
public:	
    AnalogInput(int pin, const char* name, uint8_t _taskId, Period_t period, Priority_t priority) : Task(name, _taskId, period, priority), DebugPrint(getTaskName()), taskId(_taskId) {}

	void initialize() override {
        outStream() << getInfoTask() << endl;
	}

	void run() override {
		outStream() << "Period: [" << (uint16_t)getPeriod() << "] ms, priority': [" << NAME_PRIORITY(getPriority()) << "]" << endl;
	}

    void handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) override {
    }

private:
    uint32_t count=0;
    uint8_t taskId;
};


#endif
