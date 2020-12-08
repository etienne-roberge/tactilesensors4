/****************************************************************************************
//DataListener V4.1
//Author: Jean-Philippe Roberge Ing, M.Sc.A.
//Date: June 9th 2015
//Affiliations: Laboratoire de commande et de robotique (École de technologie supérieure)
//
//Description:  DataListener.cpp - A example script that listens for any kind of data
//              coming from any sensor(s). This starts by subscribing to the topics
//              "/TactileSensor/DynamicData" and "/TactileSensor/DynamicData". It will
//              display any data it receives.
//
//______________________________________________________________________________________
//Version: 1.0 : April 2nd 2015
//Version: 1.1 : June 9th 2015
//
//Last Modified: April 2nd 2015 - Initial release
//               June 9th 2015  - Modified to include acquisition of multiple sensors at
//                                the same time --> e.g. by adding option -sensor 1:10
//                                on the command line
****************************************************************************************/



#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"
#include "tactilesensors4/StaticData.h"
#include "tactilesensors4/Dynamic.h"
#include "tactilesensors4/Accelerometer.h"

enum DataType
{
    StaticData,
    DynamicData,
    Accel,
    Euler,
    Gyro,
    Magneto,
    Quaternion
};

typedef std::vector<DataType> DataTypeV;

void show_usage();
DataTypeV readCmdOptions(int argc, char **argv);


//void staticdataCallback(const tactilesensors4::StaticData::ConstPtr &msg)
//{
//    float sensor1[28];
//    float sensor2[28];
//    float maxValue = 0;

//    float stat1[28] = {16620, 13418, 17784, 11843, 21421, 15800, 11099, 11405, 26420, 10090, 10363, 10546, 35619, 9862, 9892, 10220, 35418, 11061, 13749, 16137, 11674, 36309, 11245, 15912, 37536, 12034, 11436, 17542};
//    float stat2[28] = {16698, 13663, 18093, 11328, 22293, 16158, 10845, 11002, 28232, 9812, 10103, 10175, 36931, 9583, 9577, 9846, 36746, 11245, 14098, 16982, 11889, 37356, 11427, 16790, 23343, 36141, 62534, 49818};

//    for(int i=0;i<msg->taxels[0].values.size();i++)
//    {
//        //ROS_INFO("We have received static data: [%i] and [%i] from the sensors", msg->taxels[0].values[i],msg->taxels[1].values[i]);
//        sensor1[i] =  (msg->taxels[0].values[i]) - stat1[i];
//        sensor2[i] =  (msg->taxels[1].values[i]) - stat2[i];
//        if(sensor1[i] > maxValue)
//        {
//            maxValue=sensor1[i];
//        }
//        if(sensor2[i] > maxValue)
//        {
//            maxValue=sensor2[i];
//        }
//    }

//    for(int j=0;j<28;j++)
//    {
//       sensor1[j] = float(sensor1[j] / maxValue);
//       sensor2[j] = float(sensor2[j] / maxValue);

//    }

//    cv::Mat Sensor1 = cv::Mat(7, 4, CV_32FC1, sensor1);
//    cv::Mat Sensor2 = cv::Mat(7, 4, CV_32FC1, sensor2); //CV_32FC1

//     cv::Mat cSensor1;
//     cvtColor(Sensor1, cSensor1,  cv::COLOR_GRAY2BGR);
//     cv::Mat cSensor2;
//     cvtColor(Sensor2, cSensor2,  cv::COLOR_GRAY2BGR);


//    cv::imshow("Sensor1", cSensor1);
//    cv::imshow("Sensor2", cSensor2);
//    cv::waitKey(50);


//}

//void dynamicdataCallback(const tactilesensors4::Dynamic::ConstPtr &msg)
//{
//    ROS_INFO("We have received dynamic data: [%i] and [%i] from the sensors", msg->data[0].value,msg->data[1].value);
//}

void accelerometerCallback(const tactilesensors4::Accelerometer::ConstPtr &msg)
{
    ROS_INFO("We have received accelerometer data from sensor #1: ax=[%i], ay=[%i] and az=[%i]", msg->data[0].values[0], msg->data[0].values[1], msg->data[0].values[2]);
//    ROS_INFO("We have received accelerometer data from sensor #2: ax=[%i], ay=[%i] and az=[%i]", msg->data[1].values[0], msg->data[1].values[1], msg->data[1].values[2]);
}

/*
//void imudataCallback(const tactilesensors4::DynamicAndIMUData::ConstPtr &msg)
//{

//    ROS_INFO("We have received dynamic+imu-->dynamic data from sensor 1: [%f]", msg->DynamicDataS1);
//    ROS_INFO("We have received dynamic+imu-->dynamic data from sensor 2: [%f]", msg->DynamicDataS2);

//    // Displaing Raw Data:
//    for(int i=0;i<msg->IMU1RawData.size();i++)
//    {
//        ROS_INFO("We have received  dynamic+imu-->raw data from sensor 1: [%f]", msg->IMU1RawData[i]);
//    }
//    for(int i=0;i<msg->IMU2RawData.size();i++)
//    {
//        ROS_INFO("We have received  dynamic+imu-->raw data from sensor 2: [%f]", msg->IMU2RawData[i]);
//    }
//    ROS_INFO("We have received  dynamic+imu-->raw data from sensor 2y: [%f]", msg->IMU2RawData[4]);
//    // Displaying Quaternions:
//    for(int i=0;i<msg->IMU1Quaternions.size();i++)
//    {
//        ROS_INFO("We have received  dynamic+imu-->quaternions data from sensor 1: [%f]", msg->IMU1Quaternions[i]);
//    }
//    for(int i=0;i<msg->IMU2Quaternions.size();i++)
//    {
//        ROS_INFO("We have received  dynamic+imu-->quaternions data from sensor 2: [%f]", msg->IMU2Quaternions[i]);
//    }

//    //Displaying Euler Angles:
//    for(int i=0;i<msg->IMU1EulerAngles.size();i++)
//    {
//        ROS_INFO("We have received  dynamic+imu-->euler data from sensor 1: [%f]", msg->IMU1EulerAngles[i]);
//    }
//    for(int i=0;i<msg->IMU2EulerAngles.size();i++)
//    {
//        ROS_INFO("We have received  dynamic+imu-->euler data from sensor 2: [%f]", msg->IMU2EulerAngles[i]);
//    }

//}*/


int main(int argc, char **argv)
{
    DataTypeV dataToPrint = readCmdOptions(argc, argv);

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);
//    ros::Subscriber sub = n.subscribe("/TactileSensor4/StaticData", 1000, staticdataCallback);
//    ros::Subscriber sub2 = n.subscribe("/TactileSensor4/Dynamic", 1000, dynamicdataCallback);
    ros::Subscriber sub_accelerometer = n.subscribe("/TactileSensor4/Accelerometer",1000,accelerometerCallback);
//    ros::Subscriber sub3 = n.subscribe("/TactileSensor4/DynamicAndIMUData", 1000, imudataCallback);
//    while (ros::ok())
//    {
//      ros::spinOnce();
//      loop_rate.sleep();
//    }
    ros::spin();

    return 0;
}

DataTypeV readCmdOptions(int nbArg, char **argv)
{
    DataTypeV printDataType;

    if(nbArg==1) // default value if no arg
    {
        printDataType.push_back(DataType::StaticData);
    }
    else // if argument, start parsing
    {
        for(int i=1; i<nbArg; ++i)
        {
            std::string arg = argv[i];
            if((arg == "-h") || (arg == "--help"))
            {
                printDataType.clear();
                show_usage();
                break;
            }
            else if((arg == "-s") || (arg == "--static"))
            {
                printDataType.push_back(DataType::StaticData);
            }
            else if((arg == "-d") || (arg == "--dynamic"))
            {
                printDataType.push_back(DataType::DynamicData);
            }
            else if((arg == "-a") || (arg == "--accelerometer"))
            {
                printDataType.push_back(DataType::Accel);
            }
            else if((arg == "-e") || (arg == "--euler"))
            {
                printDataType.push_back(DataType::Euler);
            }
            else if((arg == "-g") || (arg == "--gyroscope"))
            {
                printDataType.push_back(DataType::Gyro);
            }
            else if((arg == "-m") || (arg == "--magnetometer"))
            {
                printDataType.push_back(DataType::Magneto);
            }
            else if((arg == "-q") || (arg == "--quaternion"))
            {
                printDataType.push_back(DataType::Quaternion);
            }
            else if((arg == "-o") || (arg == "--omg"))
            {
                printDataType.clear();
                for(int type = DataType::StaticData; type!=DataType::Quaternion; type++)
                {
                    printDataType.push_back(static_cast<DataType>(type));
                }
                break;
            }
            else //option not valid, print --help and empty printing vector. Program will exit if vector is empty
            {
                printDataType.clear();
                std::cerr << "Unrecognised option -> " << arg << std::endl;
                show_usage();
                break;
            }
        }
    }

    return printDataType;
}

void show_usage()
{
    std::cerr << "Usage: Datalistener4 <option(s)> \n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-s,--static\t\tPrint static pressure data (default)\n"
              << "\t-d,--dynamic\t\tPrint dynamic data\n"
              << "\t-a,--accelerometer\tPrint accelerometer data\n"
              << "\t-e,--euler\t\tPrint print euler angles\n"
              << "\t-g,--gyroscope\t\tPrint gyroscopes data\n"
              << "\t-m,--magnetometer\tPrint magnetometers data\n"
              << "\t-q,--quaternion\t\tPrint quaternions data\n"
              << "\t-o,--omg\t\tPrint everything (will be a mess to read...)\n"
              << std::endl;
}