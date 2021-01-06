/******************************************************************************************
//PollData V4.0 : Package for the new tactile sensor as per date of October 12th 2016
//Author: Jean-Philippe Roberge Ing, M.Sc.A.
//Creation Date: April 2nd 2015
//Affiliations: Laboratoire de commande et de robotique (École de technologie supérieure)
//
//Description:  PollData4.cpp - Source code of the ros package responsible for extracting
//              and publishing the static, dynamic and IMU data from tactile sensors. The
//              results are then published on /TactileSensor3/StaticData ,
//              /TactileSensor3/DynamicData or /TactileSensor3/DynamicAndIMUData
//              respectively. The static data are published in format
//              std_msgs::Int32MultiArray while dynamic and imu data are published in
//              format std_msgs::Float64.
//
//Synopsis:
rosrun tactilesensors PollData [-device PATH_TO_DEV]
//
//              Where [OPTIONS]:
//                  -device PATH_TO_DEV is the path to the device (Note that by default
//                  "/dev/ttyACM0" is considered to be the proper path.
//                  -data static | dynamic | dyna+imu' specifies if we want to extract
//                  "static" data, "dynamic" data or "dynamic" data along with the "imu"
//                  data. By default, this node will extract static data.
//
//Comments:     1) Magnetometers are currently unsupported, their values are thus set to
//              0 by default.
//              2) The First time one queries the imu(s), please be aware that
//              "BIASCalculationIterations" milliseconds of waiting time will be
//              required. "BIASCalculationIterations" is a global variable defined just
//              below.
//              3) During the biases calculations, it is mandatory that the imus remain
//              still, i.e.: the sensors sould not be moving / vibrating at all.
//
//
//Examples:     1)  rosrun tactilesensors PollData -sensor 1,2,5
//              -This will publish static data from sensors 1,2 and 5 to the topic
//              /TactileSensor/StaticData
//              2)  rosrun tactilesensors PollData -sensors 3:7 -data dynamic
//              -This will publish dynamic data from sensors 3, 4, 5, 6 and 7 to the
//              topic /TactileSensor/DynamicData
//
//______________________________________________________________________________________
//Version 1.0 : April 2nd 2015 - Initial release
//Version 1.1 : June 9th 2015  - Modified to include acquisition of multiple sensors at
//                               the same time --> e.g. by adding option -sensor 1:10
//                               on the command line
//
//Version 2.0 : --- n/a
//
//Version 3.0 : December 8th 2015 - Modifications to comply with the new sensor hardware
//                                  structure.
//
//Version 3.1 : December 15th 2015 - Modifications to add IMU data acquisition and
//                                   processing.
//
//Version 3.2 : June 2nd 2016 - Modifications to add combined dynamic data acquisition
//                              and processing, to fix jitter issued in dynamic and imu
//                              signals and to fix euler angle calculations. Biases
//                              estimation for accelerometers and gyroscopes are now
//                              included since it is not done on the PSoC side anymore.
//
//Version 4.0 : October 12th 2016 - Major modifications to comply with the new
//                                  communication protocol and data format. The sensor,
//                                  which has almost the same hardware as in v3.X, has a
//                                  different PSoC program which includes new and improved
//                                  communication protocol and data format. As of today,
//                                  all the data is acquired at the same time without
//                                  causing any noise problem on the dynamic channel as
//                                  before.
******************************************************************************************/

#include <cstdint>
//#include "ros/ros.h"
#include "Finger.h"
#include "tactilesensors4/TactileSensors.h"
#include "tactilesensors4/Accelerometer.h"
#include "tactilesensors4/Dynamic.h"
#include "tactilesensors4/EulerAngle.h"
#include "tactilesensors4/Gyroscope.h"
#include "tactilesensors4/Magnetometer.h"
#include "tactilesensors4/Quaternion.h"
#include "tactilesensors4/StaticData.h"
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <unistd.h>

//Using std namespace
using namespace std;

#define READ_DATA_PERIOD_MS 1
#define FINGER_COUNT 2

//Global Variables:
bool StopSensorDataAcquisition=false;

enum UsbPacketSpecial
{
    USB_PACKET_START_BYTE = 0x9A
};

enum UsbCommands
{
    USB_COMMAND_READ_SENSORS = 0x61,
    USB_COMMAND_AUTOSEND_SENSORS = 0x58,
    USB_COMMAND_ENTER_BOOTLOADER = 0xE2
};

struct UsbPacket
{
    uint8_t start_byte;
    uint8_t crc8;           // over command, data_length and data
    uint8_t command;        // 4 bits of flag (MSB) and 4 bits of command (LSB)
    uint8_t data_length;
    uint8_t data[60];
};


struct Fingers
{
    int64_t timestamp;
    std::vector<std::unique_ptr<Finger>> finger;
    Fingers()
    {
        finger.push_back(std::make_unique<Finger>(1));
        finger.push_back(std::make_unique<Finger>(2));
    }
};

//Function prototypes:
bool cmdOptionExists(char** begin, char** end, const string& option);
char* getCmdOption(char ** begin, char ** end, const string & option);
bool OpenAndConfigurePort(int *USB, char const *TheDevice);
static void usbSend(const int *USB, UsbPacket *packet);
static uint8_t calcCrc8(uint8_t *data, size_t len);
static bool usbReadByte(UsbPacket *packet, unsigned int *readSoFar, uint8_t d);
static bool parseSensors(UsbPacket *packet, Fingers *fingers);

//Callbacks:
bool TactileSensorServiceCallback(tactilesensors4::TactileSensors::Request  &req, tactilesensors4::TactileSensors::Response &res)
{
    ROS_INFO("The Tactile Sensors Service has received a request: [%s]",req.Request.data());
    if(strcmp(req.Request.data(),"start")==0 || strcmp(req.Request.data(),"Start")==0)
    {
        StopSensorDataAcquisition=false;
        res.Response=true;
        return true;
    }
    else if(strcmp(req.Request.data(),"stop")==0 || strcmp(req.Request.data(),"Stop")==0)
    {
        StopSensorDataAcquisition=true;
        res.Response=true;
        return true;
    }
    else
    {
        ROS_WARN("The Tactile Sensor service has received an invalid request.");
        res.Response=false;
        return false;
    }
}

//Main
int main(int argc, char **argv)
{
    //Variable declarations:
    ros::init(argc, argv, "PollData");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);
    ros::ServiceServer TactileSensorService=n.advertiseService("Tactile_Sensors_Service", TactileSensorServiceCallback);
    // I decided not to publish the quaternions for now (since I'm not sure it would be useful for anyone...:
    //    ros::Publisher Quaternion = n.advertise<tactilesensors4::Quaternion>("TactileSensor4/Quaternion",1000);

    int USB, n_read;
    char const * TheDevice = "/dev/ttyACM0"; // By default, the device descriptor is set to "/dev/ttyACM0"
    UsbPacket send{}, recv{};
    unsigned int recvSoFar=0;
    std::vector<char> receiveBuffer(4096);

    if(cmdOptionExists(argv, argv+argc, "-device"))
    {
        char * filename = getCmdOption(argv, argv + argc, "-device");
        if (filename)
        {
            TheDevice=filename;
        }
    }

    if(!OpenAndConfigurePort(&USB,TheDevice)) return 1;

    // Gathered data
    Fingers fingers = {};


    send.command=USB_COMMAND_AUTOSEND_SENSORS;
    send.data_length = 1;
    send.data[0] = READ_DATA_PERIOD_MS;
    usbSend(&USB, &send);

    //Now we enter the ros loop where we will acquire and publish data
    while (ros::ok())
    {

        //Read the answer, i.e. the bytes received from the sensor:
        n_read = read(USB, receiveBuffer.data(), receiveBuffer.size());

        //   Parse packets and store sensor values
        for (int64_t i = 0; i < n_read; ++i)
        {
            if (usbReadByte(&recv, &recvSoFar, receiveBuffer[i]))
            {
                parseSensors(&recv, &fingers);
            }
        }

        ros::spinOnce();    //Refresh publishing buffers
    }

    // Stop auto-send message
    send.command = USB_COMMAND_AUTOSEND_SENSORS;
    send.data_length = 1;
    send.data[0] = 0;
    usbSend(&USB, &send);

    close(USB);  // close and free /dev/ttyACM0 peripheral
    return 0;
}

/*****************************************************************************************
//Function: cmdOptionExists
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns true, otherwise it will return false.
//
*****************************************************************************************/
bool cmdOptionExists(char** begin, char** end, const string& option)
{
    return find(begin, end, option) != end;
}

/****************************************************************************************
//Function: getCmdOption
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns a pointer pointing just after the string that was found.
//
****************************************************************************************/
char* getCmdOption(char ** begin, char ** end, const string & option)
{
    char ** itr = find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return nullptr;
}

/****************************************************************************************
//Function: OpenAndConfigurePort
//
//Description:  This function opens the /dev/ttyUSB0 port with read and write access
//              rights (that suppose that the user has done "sudo chmod 777 /dev/ttyUSB0"
//              before. If it succeeds, it returns true, otherwise it returns false.
//
//Arguments:    int *USB: A pointer thats points file descriptor attached to ttyUSB0
//                          device.
//              char const * TheDevice: The string of the device we want to open.
//                          (e.g. /dev/ttyACM0).
//              int DesiredVMIN: is an integer used to determine how many byte we need to
//                          receive before we consider the message/line has ended. If set
//                          to 0, read will be non-blocking. See Termios documentation
//                          for more details.
//              int DesiredVTIME: is an integer representing the number of deciseconds
//                          before we consider that a timeout has occurred. If set to 0,
//                          then we will do pure data polling and usually, in this latter
//                          case, a timeout loop still need to be coded manually. See the
//                          Termios documentation for more details.
****************************************************************************************/
bool OpenAndConfigurePort(int *USB, char const * TheDevice)
{
    // All written by Jean-Philippe Roberge on April 2015, reviewed by Jean-Philippe Roberge in June 2016
    /* Open File Descriptor */
    //            *USB = open( "/tmp/interceptty", O_RDWR| O_NOCTTY ); // For debugging purposes (JP)
    *USB = open(TheDevice, O_RDWR| O_NOCTTY );
    /* Error Handling */
    if ( (*USB) < 0 )
    {
        cout << "Error " << errno << " opening " << TheDevice << ": " << strerror (errno) << endl;
        return false;
    }

    /**** Configure Port ****/
    struct termios tty{};
    memset(&tty, 0, sizeof tty);
    if ( tcgetattr ( (*USB), &tty ) != 0 ) {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return false;
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B115200);
    cfsetispeed (&tty, (speed_t)B115200);

    /* Setting other Port Stuff */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( (*USB), TCIFLUSH );
    if ( tcsetattr ( (*USB), TCSANOW, &tty ) != 0) {
        cout << "Error " << errno << " from tcsetattr" << endl;
        return false;
    }
    return true;
}

static void usbSend(const int *USB, UsbPacket *packet)
{
    auto *p = (uint8_t *)packet;
    int n_written;

    packet->start_byte = USB_PACKET_START_BYTE;
    packet->crc8 = calcCrc8(p + 2, packet->data_length + 2);

    n_written = write( (*USB), (char *)p, packet->data_length + 4 );
}

static uint8_t calcCrc8(uint8_t *data, size_t len)
{
    // TODO: calculate CRC8
    return data[-1];
}

static bool usbReadByte(UsbPacket *packet, unsigned int *readSoFar, uint8_t d)
{
    auto *p = (uint8_t *)packet;

    // Make sure start byte is seen
    if (*readSoFar == 0 && d != USB_PACKET_START_BYTE)
        return false;

    // Buffer the byte (making sure not to overflow the packet)
    if (*readSoFar < 64)
        p[*readSoFar] = d;
    ++*readSoFar;

    // If length is read, stop when done
    if (*readSoFar > 3 && *readSoFar >= (unsigned)packet->data_length + 4)
    {
        *readSoFar = 0;

        // If CRC is ok, we have a new packet!  Return it.
        if (packet->crc8 == calcCrc8(p + 2, packet->data_length + 2))
            return true;

        // If CRC is not ok, find the next start byte and shift the packet back in hopes of getting back in sync
        for (unsigned int i = 1; i < (unsigned)packet->data_length + 4; ++i)
            if (p[i] == USB_PACKET_START_BYTE)
            {
                memmove(p, p + i, packet->data_length + 4 - i);
                *readSoFar = packet->data_length + 4 - i;
                break;
            }
    }

    return false;
}


static bool parseSensors(UsbPacket *packet, Fingers *fingers)
{
    bool sawDynamic = false;
    bool errorFlag = false;
    for (int i = 0; i < packet->data_length;)
    {
        uint8_t sensorType = packet->data[i] & 0xF0;
        uint8_t f = packet->data[i] >> 2 & 0x03;
        ++i;

        uint8_t *sensorData = packet->data + i;
        unsigned int sensorDataBytes = packet->data_length - i;

        i += fingers->finger[f]->setNewSensorValue(sensorType, sensorData, sensorDataBytes, &errorFlag);

        if(errorFlag)
            break;

        if(sensorType == USB_SENSOR_TYPE_DYNAMIC_TACTILE)
            sawDynamic = true;
    }

    /*
     * Return true every time dynamic data is read.  This is used to identify when a whole set of data has
     * arrived and needs to be processed.
     */
    return sawDynamic;
}