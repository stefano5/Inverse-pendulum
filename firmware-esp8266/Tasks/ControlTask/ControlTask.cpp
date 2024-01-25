#pragma once

#include "ControlTask.hpp"



void ControlTask::initialize() {
    outStream() << getInfoTask() << endl;
    sendMessage((uint8_t)NameTask::Estimation, (uint32_t)EstimationTask::Message_t::GET_ANGLES_POINTER);
//void Task::sendMessage(uint8_t idReceiver, uint8_t messageType, uint32_t messageBody) {

}

void ControlTask::run() {
    ASSERT_PTR(angles);
    outStream() << "Period: [" << (uint16_t)getPeriod() << "] ms, priority': [" << NAME_PRIORITY(getPriority()) << "]" << endl;

    float r = 0.0;
    float e = 
    


}

void ControlTask::handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) {
    switch (messageType) {
        case (uint8_t)Message_t::GET_ANGLES_POINTER:
            angles = (EstimationTask::Values*)messageBody;
            break;
        default:
            FATAL_ERROR;
    }

}
