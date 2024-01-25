#pragma once

#include "EstimationTask.hpp"



void EstimationTask::initialize() {
    outStream() << getInfoTask() << endl;
    initSensors();
}

void EstimationTask::run() {
    outStream() << "Period: [" << (uint16_t)getPeriod() << "] ms, priority': [" << NAME_PRIORITY(getPriority()) << "]" << endl;
    updateMeasurements();
    angles.x_angle = getMeasure(Sensors_t::X_ANGLE);
    angles.y_angle = getMeasure(Sensors_t::Y_ANGLE);
    angles.z_angle = getMeasure(Sensors_t::Z_ANGLE);
}

void EstimationTask::handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) {
    switch (messageType) {
        case (uint8_t)Message_t::GET_ANGLES_POINTER: 
            sendMessage(taskSender, messageType, (uint32_t)&angles);
            break;
        default:
            FATAL_ERROR;
    }
}
