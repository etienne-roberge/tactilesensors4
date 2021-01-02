//
// Created by etienne on 2020-12-22.
//

#ifndef TACTILESENSORS4_FINGER_H
#define TACTILESENSORS4_FINGER_H


#include <cstdint>

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

    const uint16_t *getStaticTactile() const;
    const int16_t *getDynamicTactile() const;
    const int16_t *getAccelerometer() const;
    const int16_t *getGyroscope() const;
    const int16_t *getMagnetometer() const;
    const float *getQuaternion() const;
    const float *getEuler() const;
    int16_t getTemperature() const;

private:
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
    float q[4] = {};
    float euler[3] = {};
    bool initDone;
    int biasCalculationIteration;
    float norm_bias;

    static const float BETA;
    static const float SAMPLE_FREQ;
    static const int BIAS_CALCULATION_ITERATIONS; // Number of samples we want to collect to compute IMU biases
    static const float ACCEL_RES;
    static const float GYRO_RES;
};


#endif //TACTILESENSORS4_FINGER_H
