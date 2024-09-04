cmake_minimum_required(VERSION 3.5)
project(inertial_sense_ros)

find_package(yaml-cpp REQUIRED)

find_package(Threads)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/refLLAUpdate.srv
        srv/FirmwareUpdate.srv
        msg/GPS.msg
        msg/PreIntIMU.msg
        msg/RTKInfo.msg
        msg/GTime.msg
        msg/GlonassEphemeris.msg
        msg/GNSSObsVec.msg
        msg/RTKRel.msg
        msg/DID_INS4.msg
        msg/INL2States.msg
        msg/DID_INS2.msg
        msg/DID_INS1.msg
        msg/GPSInfo.msg
        msg/GNSSEphemeris.msg
        msg/GNSSObservation.msg
        msg/SatInfo.msg
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

include_directories(include
  ${YAML_CPP_INCLUDE_DIR}
  lib/inertial-sense-sdk/src #This line of CMakeList.txt stays in .external file to reference submodule
)

add_library(inertial_sense_ros
        src/inertial_sense_ros.cpp
)
target_include_directories(inertial_sense_ros PUBLIC include lib/inertial-sense-sdk/src)
ament_target_dependencies(inertial_sense_ros InertialSense ${YAML_CPP_LIBRARIES} pthread "geometry_msgs" "tf2_ros" "diagnostic_msgs" "nav_msgs" "std_msgs" "tf2" "std_srvs" "sensor_msgs")

add_executable(inertial_sense_node src/inertial_sense_node.cpp)
ament_target_dependencies(inertial_sense_node inertial_sense_ros "geometry_msgs" "tf2_ros" "diagnostic_msgs" "nav_msgs" "std_msgs" "tf2" "std_srvs" "sensor_msgs")
ament_export_include_directories(include)
ament_export_libraries(inertial_sense_ros)

# catkin_add_gtest(test_client_reconnect test/test_client_reconnect.cpp)
# target_link_libraries(test_client_reconnect ${PROJECT_NAME})

ament_export_dependencies(rclcpp sensor_msgs geometry_msgs)
ament_package()

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

add_subdirectory(lib/inertial-sense-sdk)
