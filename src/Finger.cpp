#include <iostream>
#include "Finger.h"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// Initialize static member of finger count
int Finger::fingerCount = 0;

const float Finger::BETA = 0.1;
const float Finger::SAMPLE_FREQ = 1000;
const int Finger::BIAS_CALCULATION_ITERATIONS = 5000;
const float Finger::ACCEL_RES = 2.0/32768.0; // Accelerometers MPU9250 set resolution
const float Finger::GYRO_RES = 250.0/32768.0; // Gyroscope set resolution (250 Degrees-Per-Second)

/****************************************************************************************
//  Finger::Finger()
//  Brief:  Finger construtor will automatically start publisher thread. It will also
//          automatically increment the sensor id for publishing with a unique name.
****************************************************************************************/
Finger::Finger():
temperature(0),
initDone(false),
biasCalculationIteration(0),
norm_bias(0),
stopThread(false)
{
    sensorId = ++fingerCount;
    q.value[0] = 1.0; //init quaternion

    publisherThread = std::thread(&Finger::runPublisher, this);
}

/****************************************************************************************
//  Finger::~Finger()
//  Brief:  Stops publisher thread before exit
****************************************************************************************/
Finger::~Finger()
{
    stopThread = true;
    if (publisherThread.joinable())
    {
        completeDataCondition.notify_all();
        publisherThread.join();
    }
}

/****************************************************************************************
//  Finger::runPublisher()
//  Brief:  Main thread of the finger object. Use to publish the sensor data to ros node.
//          When a complete set of data is received from the sensor, the thread updates
//          the quaternions and euler value before publishing
****************************************************************************************/
void Finger::runPublisher()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Rate loop_rate(1000);
    ros::Publisher staticData_pub = node->advertise<tactilesensors4::StaticData>("TactileSensor4/Sensor" + std::to_string(sensorId) + "/StaticData", 1000);
    ros::Publisher dynamic_pub = node->advertise<tactilesensors4::Dynamic>("TactileSensor4/Sensor" + std::to_string(sensorId) + "/Dynamic", 1000);
    ros::Publisher accelerometer_pub = node->advertise<tactilesensors4::Accelerometer>("TactileSensor4/Sensor" + std::to_string(sensorId) + "/Accelerometer",1000);
    ros::Publisher eulerAngle_pub = node->advertise<tactilesensors4::EulerAngle>("TactileSensor4/Sensor" + std::to_string(sensorId) + "/EulerAngle",1000);
    ros::Publisher gyroscope_pub = node->advertise<tactilesensors4::Gyroscope>("TactileSensor4/Sensor" + std::to_string(sensorId) + "/Gyroscope",1000);
    ros::Publisher magnetometer_pub = node->advertise<tactilesensors4::Magnetometer>("TactileSensor4/Sensor" + std::to_string(sensorId) + "/Magnetometer",1000);
    ros::Publisher quaternion_pub = node->advertise<tactilesensors4::Quaternion>("TactileSensor4/Sensor" + std::to_string(sensorId) + "/Quaternion",1000);

    while (ros::ok() && !stopThread)
    {
        std::unique_lock<std::mutex> lock(completeDataMutex);

        if(completeDataCondition.wait_until(lock, std::chrono::system_clock::now() + 500ms) != std::cv_status::timeout)
        {
            if(stopThread) break;

            std::unique_lock<std::mutex> publishLock(publishingMutex);

            updateIMU();

            dynamic_pub.publish(dynamic);
            staticData_pub.publish(staticData);
            accelerometer_pub.publish(accelerometer);
            gyroscope_pub.publish(gyroscope);
            magnetometer_pub.publish(magnetometer);

            if(initDone)
            {
                eulerAngle_pub.publish(eulerAngle);
                quaternion_pub.publish(q);
            }

            loop_rate.sleep();
        }
        else
        {
            std::cerr << "No data received for Sensor" << sensorId << "..." << std::endl;
        }
    }
}

/****************************************************************************************
//  Finger::updateIMU()
//  Brief:  Using the accel and gyro from the sensor, calculate the the new quaternion
//          euler angles values. If the initialisation of the sensor is not finished,
//          calculate the bias instead.
****************************************************************************************/
void Finger::updateIMU()
{
    if(initDone)
    {
        //Write the accel and gyro data to the topic Struct with the computed sensor biases
        float ax, ay, az, gx, gy, gz;
        ax = accelerometer.value[0] * ACCEL_RES - accelBias[0];
        ay = accelerometer.value[1] * ACCEL_RES - accelBias[1];
        az = accelerometer.value[2] * ACCEL_RES - accelBias[2];

        gx = gyroscope.value[0] * GYRO_RES - gyroBias[0];
        gy = gyroscope.value[1] * GYRO_RES - gyroBias[1];
        gz = gyroscope.value[2] * GYRO_RES - gyroBias[2];

        madgwickAHRSUpdateIMU(gx * M_PI / 180, gy * M_PI / 180, gz * M_PI / 180, ax, ay, az); // 6-axis IMU

        eulerAngle.value[0] = atan2(2.0f*(q.value[0]*q.value[1]+q.value[2]*q.value[3]),q.value[0]*q.value[0]-q.value[1]*q.value[1]-q.value[2]*q.value[2]+q.value[3]*q.value[3])*180/M_PI;
        eulerAngle.value[1] = -asin(2.0f*(q.value[1]*q.value[3]-q.value[0]*q.value[2]))*180/M_PI;
        eulerAngle.value[2] = atan2(2.0f*(q.value[1]*q.value[2]+q.value[0]*q.value[3]),q.value[0]*q.value[0]+q.value[1]*q.value[1]-q.value[2]*q.value[2]-q.value[3]*q.value[3])*180/M_PI;

    }
    else
    {
        initBias();
    }
}

/****************************************************************************************
//  Finger::initBias()
//  Brief:  Calculate the bias for the gyro and accel for the first
//          BIAS_CALCULATION_ITERATIONS at start.
****************************************************************************************/
void Finger::initBias()
{
    if(biasCalculationIteration < BIAS_CALCULATION_ITERATIONS)
    {
        gyroBias[0] += gyro[0] * GYRO_RES;
        gyroBias[1] += gyro[1] * GYRO_RES;
        gyroBias[2] += gyro[2] * GYRO_RES;

        accelBias[0] += accel[0] * ACCEL_RES;
        accelBias[1] += accel[1] * ACCEL_RES;
        accelBias[2] += accel[2] * ACCEL_RES;

        biasCalculationIteration++;
    }
    else
    {
        gyroBias[0] /= biasCalculationIteration;
        gyroBias[1] /= biasCalculationIteration;
        gyroBias[2] /= biasCalculationIteration;

        accelBias[0] /= biasCalculationIteration;
        accelBias[1] /= biasCalculationIteration;
        accelBias[2] /= biasCalculationIteration;

        norm_bias = sqrtf(pow(accelBias[0],2) + pow(accelBias[1],2) + pow(accelBias[2],2)) - 1;

        accelBias[0] *= norm_bias / (accelBias[0] + accelBias[1] + accelBias[2]);
        accelBias[1] *= norm_bias / (accelBias[0] + accelBias[1] + accelBias[2]);
        accelBias[2] *= norm_bias / (accelBias[0] + accelBias[1] + accelBias[2]);

        initDone = true;
    }
}

/****************************************************************************************
//  Finger::setNewSensorValue(int sensorType, uint8_t *data, unsigned int size, bool* errorFlag)
//  Brief:  updates the sensor values. Call the publishing thread when USB_SENSOR_TYPE_DYNAMIC_TACTILE
//          is seen.
//  Param:
//     - sensorType:   the sensor data type as specified in enum UsbSensorType.
//     - data:         the raw data from sensor
//     - size:         size of data
//     - error:        error flag used to return error
//
//  Return: The number of byte read
****************************************************************************************/
int Finger::setNewSensorValue(int sensorType, uint8_t *data, unsigned int size, bool* errorFlag)
{
    int byteRead;

    std::unique_lock<std::mutex> publishLock(publishingMutex);

    switch (sensorType)
    {
        case USB_SENSOR_TYPE_DYNAMIC_TACTILE:
            byteRead = extractUint16((uint16_t *)dynamicTactile, FINGER_DYNAMIC_TACTILE_COUNT, data, size);
            dynamic.value = dynamicTactile[0];
            completeDataCondition.notify_all(); //we received all the data! We can publish
            break;
        case USB_SENSOR_TYPE_STATIC_TACTILE:
            byteRead = extractUint16(staticTactile, FINGER_STATIC_TACTILE_COUNT, data, size);
            std::copy(std::begin(staticTactile), std::end(staticTactile), std::begin(staticData.value));
            break;
        case USB_SENSOR_TYPE_ACCELEROMETER:
            byteRead = extractUint16((uint16_t *)accel, 3, data, size);
            std::copy(std::begin(accel), std::end(accel), std::begin(accelerometer.value));
            break;
        case USB_SENSOR_TYPE_GYROSCOPE:
            byteRead = extractUint16((uint16_t *)gyro, 3, data, size);
            std::copy(std::begin(gyro), std::end(gyro), std::begin(gyroscope.value));
            break;
        case USB_SENSOR_TYPE_MAGNETOMETER:
            byteRead = extractUint16((uint16_t *)magnet, 3, data, size);
            std::copy(std::begin(magnet), std::end(magnet), std::begin(magnetometer.value));
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

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//---------------------------------------------------------------------------------------------------
void Finger::madgwickAHRSUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quat from gyro
    qDot1 = 0.5f * (-q.value[1] * gx - q.value[2] * gy - q.value[3] * gz);
    qDot2 = 0.5f * (q.value[0] * gx + q.value[2] * gz - q.value[3] * gy);
    qDot3 = 0.5f * (q.value[0] * gy - q.value[1] * gz + q.value[3] * gx);
    qDot4 = 0.5f * (q.value[0] * gz + q.value[1] * gy - q.value[2] * gx);

    // Compute feedback only if accel measurement valid (avoids NaN in accel normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accel measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q.value[0];
        _2q1 = 2.0f * q.value[1];
        _2q2 = 2.0f * q.value[2];
        _2q3 = 2.0f * q.value[3];
        _4q0 = 4.0f * q.value[0];
        _4q1 = 4.0f * q.value[1];
        _4q2 = 4.0f * q.value[2];
        _8q1 = 8.0f * q.value[1];
        _8q2 = 8.0f * q.value[2];
        q0q0 = q.value[0] * q.value[0];
        q1q1 = q.value[1] * q.value[1];
        q2q2 = q.value[2] * q.value[2];
        q3q3 = q.value[3] * q.value[3];

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.value[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q.value[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q.value[3] - _2q1 * ax + 4.0f * q2q2 * q.value[3] - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;
    }

    // Integrate rate of change of quat to yield quat
    q.value[0] += qDot1 * (1.0f / SAMPLE_FREQ);
    q.value[1] += qDot2 * (1.0f / SAMPLE_FREQ);
    q.value[2] += qDot3 * (1.0f / SAMPLE_FREQ);
    q.value[3] += qDot4 * (1.0f / SAMPLE_FREQ);

    // Normalise quat
    recipNorm = invSqrt(q.value[0] * q.value[0] + q.value[1] * q.value[1] + q.value[2] * q.value[2] + q.value[3] * q.value[3]);
    q.value[0] *= recipNorm;
    q.value[1] *= recipNorm;
    q.value[2] *= recipNorm;
    q.value[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float Finger::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

inline uint16_t Finger::parseBigEndian2(const uint8_t *data)
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
