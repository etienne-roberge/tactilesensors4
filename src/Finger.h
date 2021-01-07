//
// Created by etienne on 2020-12-22.
//

#ifndef TACTILESENSORS4_FINGER_H
#define TACTILESENSORS4_FINGER_H


#include <cstdint>
#include <ros/ros.h>
#include <tactilesensors4/Quaternion.h>
#include <tactilesensors4/Dynamic.h>
#include <tactilesensors4/Accelerometer.h>
#include <tactilesensors4/EulerAngle.h>
#include <tactilesensors4/Magnetometer.h>
#include <tactilesensors4/StaticData.h>
#include <tactilesensors4/Gyroscope.h>
#include <thread>
#include <condition_variable>

#define FINGER_STATIC_TACTILE_ROW 4
#define FINGER_STATIC_TACTILE_COL 7
#define FINGER_STATIC_TACTILE_COUNT (FINGER_STATIC_TACTILE_ROW * FINGER_STATIC_TACTILE_COL)
#define FINGER_DYNAMIC_TACTILE_COUNT 1

// Sensor types occupy the higher 4 bits, the 2 bits lower than that identify finger, and the lower 2 bits is used as an index.
enum UsbSensorType
{
    USB_SENSOR_TYPE_STATIC_TACTILE = 0x10,
    USB_SENSOR_TYPE_DYNAMIC_TACTILE = 0x20,
    USB_SENSOR_TYPE_ACCELEROMETER = 0x30,
    USB_SENSOR_TYPE_GYROSCOPE = 0x40,
    USB_SENSOR_TYPE_MAGNETOMETER = 0x50,
    USB_SENSOR_TYPE_TEMPERATURE = 0x60
};

class Finger {
public:
    Finger();
    virtual ~Finger();

    void update();
    int setNewSensorValue(int sensorType, uint8_t *data, unsigned int size, bool* errorFlag);

private:
    void runPublisher();
    void updateIMU();
    void initBias();
    void madgwickAHRSUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    static uint8_t extractUint16(uint16_t *to, uint16_t toCount, uint8_t *data, unsigned int size);
    static inline uint16_t parseBigEndian2(uint8_t *data);
    static float invSqrt(float x);

    uint16_t staticTactile[FINGER_STATIC_TACTILE_COUNT] = {};
    int16_t dynamicTactile[FINGER_DYNAMIC_TACTILE_COUNT] = {};
    int16_t accel[3] = {};
    float accelBias[3] = {};
    int16_t gyro[3] = {};
    float gyroBias[3] = {};
    int16_t magnet[3] = {};
    int16_t temperature;
    bool initDone;
    int biasCalculationIteration;
    float norm_bias;

    int sensorId;

    tactilesensors4::Quaternion q;
    tactilesensors4::Dynamic dynamic;
    tactilesensors4::Accelerometer accelerometer;
    tactilesensors4::EulerAngle eulerAngle;
    tactilesensors4::Magnetometer magnetometer;
    tactilesensors4::StaticData staticData;
    tactilesensors4::Gyroscope gyroscope;

    std::thread publisherThread;
    std::condition_variable completeDataCondition;
    std::mutex completeDataMutex;
    std::mutex publishingMutex;
    bool stopThread;

    static const float BETA;
    static const float SAMPLE_FREQ;
    static const int BIAS_CALCULATION_ITERATIONS; // Number of samples we want to collect to compute IMU biases
    static const float ACCEL_RES;
    static const float GYRO_RES;

    static int fingerCount;
};


#endif //TACTILESENSORS4_FINGER_H
