//
// Created by etienne on 2020-12-22.
//

#include <iostream>
#include "Finger.h"

Finger::Finger()
{

    uint16_t staticTactile[FINGER_STATIC_TACTILE_COUNT];
    int16_t dynamicTactile[FINGER_DYNAMIC_TACTILE_COUNT];
    int16_t accelerometer[3];
    int16_t gyroscope[3];
    int16_t magnetometer[3];
    int16_t temperature;
}

Finger::~Finger() {

}

int Finger::setNewSensorValue(int sensorType, uint8_t *data, unsigned int size, bool* errorFlag)
{
    int byteRead;
    switch (sensorType)
    {
        case USB_SENSOR_TYPE_DYNAMIC_TACTILE:
            byteRead = extractUint16((uint16_t *)dynamicTactile, FINGER_DYNAMIC_TACTILE_COUNT, data, size);
            break;
        case USB_SENSOR_TYPE_STATIC_TACTILE:
            byteRead = extractUint16(staticTactile, FINGER_STATIC_TACTILE_COUNT, data, size);
            break;
        case USB_SENSOR_TYPE_ACCELEROMETER:
            byteRead = extractUint16((uint16_t *)accelerometer, 3, data, size);
            break;
        case USB_SENSOR_TYPE_GYROSCOPE:
            byteRead = extractUint16((uint16_t *)gyroscope, 3, data, size);
            break;
        case USB_SENSOR_TYPE_MAGNETOMETER:
            byteRead = extractUint16((uint16_t *)magnetometer, 3, data, size);
            break;
        case USB_SENSOR_TYPE_TEMPERATURE:
            byteRead = extractUint16((uint16_t *)&temperature, 1, data, size);
            break;
        default:
            // Unknown sensor, we can't continue parsing anything from here on
            std::cerr << "Unrecognised sensor type -> " << sensorType << std::endl;
            *errorFlag = true;
            byteRead = -1;
    }
    return byteRead;
}


inline uint16_t Finger::parseBigEndian2(uint8_t *data)
{
    return (uint16_t)data[0] << 8 | data[1];
}

uint8_t Finger::extractUint16(uint16_t *to, uint16_t toCount, uint8_t *data, unsigned int size)
{
    unsigned int cur;

    // Extract 16-bit values.  If not enough data, extract as much data as available
    for (cur = 0; 2 * cur + 1 < size && cur < toCount; ++cur)
        to[cur] = parseBigEndian2(&data[2 * cur]);

    // Return number of bytes read
    return cur * 2;
}

const uint16_t *Finger::getStaticTactile() const {
    return staticTactile;
}

const int16_t *Finger::getDynamicTactile() const {
    return dynamicTactile;
}

const int16_t *Finger::getAccelerometer() const {
    return accelerometer;
}

const int16_t *Finger::getGyroscope() const {
    return gyroscope;
}

const int16_t *Finger::getMagnetometer() const {
    return magnetometer;
}

int16_t Finger::getTemperature() const {
    return temperature;
}
