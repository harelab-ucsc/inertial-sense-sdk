cmake_minimum_required(VERSION 3.10.0)
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

# We don't need the example projects
set(IGNORE_EXAMPLE_PROJECTS 1)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/refLLAUpdate.srv
        srv/FirmwareUpdate.srv
        msg/GPS.msg
        msg/RTKInfo.msg
        msg/GTime.msg
        msg/GlonassEphemeris.msg
        msg/PIMU.msg
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

include_directories(include ${YAML_CPP_INCLUDE_DIR} ../src #This line of CMakeList.txt stays in .external file to reference submodule
 ../src/libusb/libusb ../src/libusb/linux ../src/yaml-cpp
)

add_library(inertial_sense_ros
    src/ParamHelper.cpp
    src/RtkRover.cpp
    src/RtkBase.cpp
    src/inertial_sense_ros.cpp
)
target_include_directories(inertial_sense_ros PUBLIC include)

ament_target_dependencies(inertial_sense_ros InertialSenseSDK ${YAML_CPP_LIBRARIES} pthread "tf2_ros" "geometry_msgs" "std_msgs" "sensor_msgs" "tf2" "std_srvs" "diagnostic_msgs" "nav_msgs")

add_executable(inertial_sense_node src/inertial_sense_node.cpp)
ament_target_dependencies(inertial_sense_node inertial_sense_ros "tf2_ros" "geometry_msgs" "std_msgs" "sensor_msgs" "tf2" "std_srvs" "diagnostic_msgs" "nav_msgs")
ament_target_dependencies(test_unit_tests ${PROJECT_NAME})
ament_target_dependencies(test_ros_bridge ${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(inertial_sense_ros)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs)
ament_package()

add_subdirectory(.. devel)

# Build using C11 and CXX20
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

ament_add_gtest(test_unit_tests test/test_basic_unit_tests.cpp)

ament_add_gtest(test_ros_bridge test/test_main.cpp test/test_communications.cpp test/test_client_reconnect.cpp)

# catkin_add_gtest(test_client_reconnect test/test_client_reconnect.cpp)
# target_link_libraries(test_client_reconnect ${PROJECT_NAME})
