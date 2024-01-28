#pragma once

#include "ControlTask.hpp"


void ControlTask::initialize() {
    ASSERT_PTR(motor_left);     // bisogna istanziare i pin dei motori
    ASSERT_PTR(motor_right);
    
    outStream() << getInfoTask() << endl;
    sendMessage((uint8_t)NameTask::Estimation, (uint32_t)EstimationTask::Message_t::GET_ANGLES_POINTER);
//void Task::sendMessage(uint8_t idReceiver, uint8_t messageType, uint32_t messageBody) {

    float kp = 15.0;
    float ki = 0.0;
    float kd = 1.0;
    float period = ((float)getPeriod()) / 1000.0;
    float sat_max = 100;
    float sat_min = -100;

    motor_left->initHW();
    motor_right->initHW();
    
    controller_left.initController(period, kp, ki, kd, sat_max, sat_min, 1.0);
    controller_right.initController(period, kp, ki, kd, sat_max, sat_min, 1.0);
}

void ControlTask::run() {
    ASSERT_PTR(angles);
    //outStream() << "Period: [" << (uint16_t)getPeriod() << "] ms, priority': [" << NAME_PRIORITY(getPriority()) << "]" << endl;

    float r = 0.0;          // da configurare eventuali offset
    float y = angles->y_angle;
    float e = r - y;

    float u_left = controller_left.get_u(e);
    float u_right = controller_right.get_u(e);
    
    outStream() << "u left: " << u_left << endl;
    outStream() << "u right: " << u_right << endl;

    motor_left->setSpeed(u_left);
    motor_right->setSpeed(u_right);
}

void ControlTask::handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody) {
    switch (messageType) {
        case (uint8_t)EstimationTask::Message_t::GET_ANGLES_POINTER:
            angles = (EstimationTask::Values*)messageBody;
            break;
        default:
            FATAL_ERROR;
    }

}
