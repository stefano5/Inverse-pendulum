#pragma once

class PIDController {
public:
    PIDController() : integrativeTerm(0.0), e_km1(0.0), stepTime(0.0), kp(0.0), ki(0.0), kd(0.0), max_u(100.0), min_u(0.0), Ta(1.0) { }

    void initController(float stepDerTime, float kp, float ki, float kd, float max_u, float min_u, float Ta);

    float proportionalTerm(float e);
    float integralTerm(float e, float u_prev, float u_sat_prev);
    float derivativeTerm(float e);
    float saturation(float u);
    float antiWindup(float u, float u_sat);
    float get_u(float error);

private:
    float integrativeTerm;
    float e_km1;

    // parametri da valorizzare:
    float stepTime;
    float kp;
    float ki;
    float kd;
    float max_u;
    float min_u;
    float Ta;   // velocità di scarico dell'integratore nell'anti windup
};

