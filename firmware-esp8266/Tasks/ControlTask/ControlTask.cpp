#pragma once

#include "ControlTask.hpp"


void ControlTask::initialize() {
    ASSERT_PTR(motor_left);     // bisogna istanziare i pin dei motori
    ASSERT_PTR(motor_right);
    
    outStream() << getInfoTask() << endl;
    sendMessage((uint8_t)NameTask::Estimation, (uint32_t)EstimationTask::Message_t::GET_ANGLES_POINTER);
//void Task::sendMessage(uint8_t idReceiver, uint8_t messageType, uint32_t messageBody) {

    float kp = 30.0;
    float ki = 0.0;
    float kd = 0.0;
    float period = ((float)getPeriod()) / 1000.0;
    float sat_max = 100;
    float sat_min = -100;

    motor_left->initHW();
    motor_right->initHW();
    
    controller_left.initController(period, kp, ki, kd, sat_max, sat_min, 1.0);
    controller_right.initController(period, kp, ki, kd, sat_max, sat_min, 1.0);


    kp = 0;
    ki = 0;

    controller_left_speed.initController(period, kp, ki, 0, 100, -100, 1);
    controller_right_speed.initController(period, kp, ki, 0, 100, -100, 1);
}

void ControlTask::run() {
    ASSERT_PTR(angles);
    //outStream() << "Period: [" << (uint16_t)getPeriod() << "] ms, priority': [" << NAME_PRIORITY(getPriority()) << "]" << endl;

    float r = 3.0;          // da configurare eventuali offset
    float y = angles->y_angle;
    float e = r - y;
    float y_dot = angles->pitch_v_ang;
    float r_dot = 0.0; 
    float e_dot = r_dot - y_dot;


    float u_left =  controller_left.get_u(e) + controller_left_speed.get_u(e_dot);
    float u_right = controller_right.get_u(e) + controller_left_speed.get_u(e_dot);
    
    outStream() << "u left: " << u_left << endl;
    outStream() << "u right: " << u_right << endl;

    if (y > 50 || y < -50) {
        outStream() << "inibhit" << endl;
        motor_left->setSpeed(0);
        motor_right->setSpeed(0);

    } else {
        motor_left->setSpeed(u_left);
        motor_right->setSpeed(u_right);

    }

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
