#pragma once

#include "PIDController.hpp"


void PIDController::initController(float stepDerTime, float kp, float ki, float kd, float max_u, float min_u, float Ta) {
    FATAL_ERROR_IF_TRUE(stepDerTime == 0);  // non può essere 0
    FATAL_ERROR_IF_TRUE(Ta == 0);           // non ppuò essere 0
    FATAL_ERROR_IF_FALSE(max_u > min_u);    // max_u must be greater then min_u

    this->stepTime = stepDerTime;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->max_u = max_u;
    this->min_u = min_u;
    this->Ta = Ta;
}


float PIDController::proportionalTerm(float e) {
    //Serial.print("kp: ");
    //Serial.println(kp);
    return e * kp;
}

float PIDController::integralTerm(float e, float u_prev, float u_sat_prev) {
    e = e + antiWindup(u_prev, u_sat_prev);

    integrativeTerm += e * ki * stepTime;
    
    return integrativeTerm;
}

float PIDController::derivativeTerm(float e) {
    FATAL_ERROR_IF_TRUE(stepTime == 0);
    float der = (e - e_km1) / stepTime;
    e_km1 = e;
    return der * kd;
}

float PIDController::saturation(float u) {
    if (u > max_u) u = max_u;
    else if (u < min_u) u = min_u;

    return u;
}

float PIDController::antiWindup(float u, float u_sat) {
    //float Ta = 1; // velocità di scarico dell'integratore
    FATAL_ERROR_IF_TRUE(Ta == 0); // non può essere 0
    float Fs = 1.0 / Ta;
    return (u - u_sat) * Fs;
}


float PIDController::get_u(float error) {
    static float u_prev = 0.0;
    static float u_sat_prev = 0.0;

    float u = proportionalTerm(error) + integralTerm(error, u_prev, u_sat_prev) + derivativeTerm(error);
    float u_sat = saturation(u);

    u_prev = u;
    u_sat_prev = u_sat;
    
    return u_sat;
}
