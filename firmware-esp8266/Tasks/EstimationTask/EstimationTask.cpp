#pragma once

#include "EstimationTask.hpp"



void EstimationTask::initialize() {
    outStream() << getInfoTask() << endl;
    
    initSensors();
}

void EstimationTask::run() {
    //outStream() << "Period: [" << (uint16_t)getPeriod() << "] ms, priority': [" << NAME_PRIORITY(getPriority()) << "]" << endl;
    
    updateMeasurements();
    angles.x_angle = getMeasure(Sensors_t::X_ANGLE);
    angles.y_angle = getMeasure(Sensors_t::Y_ANGLE) + 90;
    angles.z_angle = getMeasure(Sensors_t::Z_ANGLE);
    angles.pitch_acc = getMeasure(Sensors_t::ACC_Y);
    angles.pitch_v_ang = getMeasure(Sensors_t::GYRO_Y);
    angles.pitch_acc_angular = getMeasure(Sensors_t::ANG_ACC_Y);

    if (debugPrint) {
        outStream() << "pitch: " << angles.y_angle << "; pitch rate: " << angles.pitch_v_ang << "; pitch acc: " << angles.pitch_acc << endl;
    }
}

void EstimationTask::handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) {
    switch (messageType) {
        case (uint8_t)Message_t::GET_ANGLES_POINTER: 
            sendMessage(taskSender, messageType, (uint32_t)&angles);
            break;
        case (uint8_t)EstimationTask::Message_t::TOGGLE_DEBUG_PRINT:
            debugPrint = !debugPrint;
            break;
        default:
            FATAL_ERROR;
    }
}
