/*
 * cpp_main.cpp
 *
 *  Created on: Gen 25, 2024
 *      Author: Maugeri
 */
// arduino
#include <Arduino.h>
#include <Ticker.h>

/// framework
#include "GlobalDependacies.hpp"
#include "ErrorsHandle/ProgrammingErrors.cpp"
#include "Serial/FWSerial.cpp"
#include "Serial/DebugPrint.cpp"

// hardware
#include "Hardware/GPIO/AnalogPin.cpp"
#include "Hardware/GPIO/GPIO.cpp"
#include "Hardware/H-Bridge/HBridge.hpp"

#include "Scheduler/Message.cpp"
#include "Scheduler/Scheduler.cpp"
#include "Scheduler/Task.cpp"
#include "Scheduler/TaskCommunication.cpp"

#include "Tasks/ExampleTask/AnalogInput.hpp"
#include "Tasks/SerialInputManager/SerialInputManager.cpp"
#include "Tasks/ControlTask/ControlTask.cpp"
#include "Tasks/EstimationTask/EstimationTask.cpp"

Ticker timer;

uint8_t flag_1ms_elapsed;

void onTimerInterrupt() {
  flag_1ms_elapsed = 1;  
}

// hardware
AnalogPin pwm_left(D5, OUTPUT, 0, 100);
AnalogPin pwm_right(D6, OUTPUT, 0, 100);
GPIO dir_left(D0, GPIO::IOMode::OUTPUT_MODE);
GPIO dir_right(D7, GPIO::IOMode::OUTPUT_MODE);
HBridge motor_left(&pwm_left, &dir_left);
HBridge motor_right(&pwm_right, &dir_right);

// task handler
Scheduler scheduler;
//AnalogInput exampleTask(myPin.pin, "Example task", (uint8_t)NameTask::exampleTask, Task::Period_t::T_100ms, Task::Priority_t::low); // example
ControlTask controlTask(&motor_left, &motor_right, "Control task", (uint8_t)NameTask::Control, Task::Period_t::T_100ms, Task::Priority_t::medium);
EstimationTask estimationTask("Estimation task", (uint8_t)NameTask::Estimation, Task::Period_t::T_100ms, Task::Priority_t::high);
SerialInputManager serialT("Serial Manager", (uint8_t)NameTask::SerialManager, Task::Period_t::T_50ms, Task::Priority_t::high);     // this task receive serial messages

void cpp_main(void) {
  FWSerial::setHandle(&Serial);


  scheduler.setup();
  flag_1ms_elapsed = 0;

  timer.attach(0.001, onTimerInterrupt);  // set 1ms timer

  while (1) {
    scheduler.runTasks(&flag_1ms_elapsed);  // scheduler lib needs to know when a 1ms has gone by
    yield();    // it is needed for esp8266
  }

	FATAL_ERROR;	// unreachable statement
}

// put here the required callback routines, if needed
void cpp_wwdg_callback(void) {
	FATAL_ERROR;
}
