#ifndef SENSORS_HANDLER_HPP
#define SENSORS_HANDLER_HPP

#include "MPU6050/MPU6050_tockn.cpp"



class SensorsHandler {
public:
    enum class Sensors_t {
        X_ANGLE=0,
        Y_ANGLE,
        Z_ANGLE,
        ACC_X,
        ACC_Y,
        ACC_Z,
        GYRO_X,
        GYRO_Y,
        GYRO_Z,
        ANG_ACC_X,
        ANG_ACC_Y,
        ANG_ACC_Z,
    };

    SensorsHandler() : mpu6050(Wire) { }
    //SensorsHandler(uint8_t i2cAddress) : mpu6050(Wire) { }

    void initSensors();

	float getMeasure(SensorsHandler::Sensors_t ch);
	bool isSensorOk();
    void updateMeasurements();


private:

    MPU6050 mpu6050;
    //uint8_t i2cAddress;

};

#endif