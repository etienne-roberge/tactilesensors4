cmake_minimum_required(VERSION 2.8.3)
project(tactilesensors4)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        genmsg
        message_generation
        cv_bridge)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   StaticData.msg
   Dynamic.msg
   Accelerometer.msg
   Gyroscope.msg
   Magnetometer.msg
   Quaternion.msg
   EulerAngle.msg
   Taxels.msg
   Data1.msg
   Data3.msg
   Data3f.msg
   Data4f.msg
   Sensor.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   TactileSensors.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

#############
## Install ##
#############

link_directories(/usr/local/lib)
find_package(PkgConfig REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-ggdb -O3 -fPIC -std=c++0x -march=native -mno-avx") #jk added

add_executable(PollData4 src/PollData4.cpp src/Finger.cpp src/Finger.h)
target_link_libraries(PollData4 ${catkin_LIBRARIES})
add_dependencies(PollData4 tactilesensors4_generate_messages_cpp)

add_executable(DataListener4 src/DataListener4.cpp)
target_link_libraries(DataListener4 ${catkin_LIBRARIES})
add_dependencies(DataListener4 tactilesensors4_generate_messages_cpp)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )