#include "SensorsHandler.hpp"


void SensorsHandler::initSensors() {
    
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    FATAL_ERROR_IF_FALSE(isSensorOk());     // il sensore non è collegato / ha un indirizzo errato

}

void SensorsHandler::updateMeasurements() {
    if (isSensorOk()) {
        mpu6050.update();
    } else {
        /* che famo? */
    }
}

float SensorsHandler::getMeasure(SensorsHandler::Sensors_t ch) {
    switch (ch) {
        case Sensors_t::X_ANGLE:
            return mpu6050.getAngleX();
        case Sensors_t::Y_ANGLE:
            return mpu6050.getAngleY();
        case Sensors_t::Z_ANGLE:
            return mpu6050.getAngleZ();
        case Sensors_t::ACC_X:
            return mpu6050.getAccX();
        case Sensors_t::ACC_Y:
            return mpu6050.getAccY();
        case Sensors_t::ACC_Z:
            return mpu6050.getAccZ();
        case Sensors_t::GYRO_X:
            return mpu6050.getGyroX();
        case Sensors_t::GYRO_Y:
            return mpu6050.getGyroY();
        case Sensors_t::GYRO_Z:
            return mpu6050.getGyroZ();
        case Sensors_t::ANG_ACC_X:
            return mpu6050.getAccAngleX();
        case Sensors_t::ANG_ACC_Y:
            return mpu6050.getAccAngleY();
        default:
            FATAL_ERROR;
            return 0;
    }
}

bool SensorsHandler::isSensorOk() {
    ASSERT_PTR(&Wire);

    Wire.beginTransmission(MPU6050_ADDR);
    return Wire.endTransmission() == 0; // true : ok, false: address not found
}

