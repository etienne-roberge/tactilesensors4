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
//              still, i.e.: the sensors should not be moving / vibrating at all.
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

#include "ros/ros.h"
#include "tactilesensors4/TactileSensors.h"
#include "Communication.h"
#include <algorithm>
#include <unistd.h>

#define DEVICE_OPTION "-device"

//Using std namespace
using namespace std;

//Function prototypes:
bool cmdOptionExists(char** begin, char** end, const string& option);
std::vector<std::string> getDevicesOption(char ** begin, char ** end, const string& option);

std::vector<std::unique_ptr<Communication>> commList;

//Callbacks:
bool TactileSensorServiceCallback(tactilesensors4::TactileSensors::Request  &req, tactilesensors4::TactileSensors::Response &res)
{
    ROS_INFO("The Tactile Sensors Service has received a request: [%s]",req.Request.data());
    if(strcmp(req.Request.data(),"start")==0 || strcmp(req.Request.data(),"Start")==0)
    {
        for(auto &&comm : commList)
        {
            comm.get()->enableComm(true);
        }

        res.Response=true;
        return true;
    }
    else if(strcmp(req.Request.data(),"stop")==0 || strcmp(req.Request.data(),"Stop")==0)
    {
        for(auto &&comm : commList)
        {
            comm.get()->enableComm(false);
        }
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
    ros::ServiceServer TactileSensorService=n.advertiseService("Tactile_Sensors_Service", TactileSensorServiceCallback);

    std::vector<std::string> devices;

    if(cmdOptionExists(argv, argv+argc, DEVICE_OPTION))
    {
        devices = getDevicesOption(argv, argv + argc, DEVICE_OPTION);
        if(devices.empty())
        {
            std::cout << "no devices specified!" << std::endl;
            return 1;
        }
    }
    else
    {
        devices.emplace_back("/dev/ttyACM0"); // By default, the device descriptor is set to "/dev/ttyACM0"
    }

    for(auto name : devices)
    {
        commList.emplace_back(std::make_unique<Communication>(&name));
    }

    //Now we enter the ros loop where we will acquire and publish data
    if (ros::ok())
    {
        ros::spin();
    }

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
//Function: getDevicesOption
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns all the arguments after the options
//
****************************************************************************************/
std::vector<std::string> getDevicesOption(char ** begin, char ** end, const string& option)
{
    std::vector<std::string> devices;
    char ** itr = find(begin, end, option);
    while (itr != end && ++itr != end)
    {
        devices.emplace_back(*itr);
    }
    return devices;
}