cmake_minimum_required(VERSION 3.0.2)
project(ekf)

set(CMAKE_BUILD_TYPE "Release")
option(EKF_ENABLE_NATIVE_OPTIMIZATION "Enable -march=native for host-specific builds" OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DEIGEN_DONT_PARALLELIZE")
if(EKF_ENABLE_NATIVE_OPTIMIZATION)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

catkin_package(
    )

add_executable(ekf src/ekf_node_vio_timesync_with_acc_pub.cpp src/conversion.cpp)
add_executable(imu_add_noise_quat src/imu_add_noise.cpp)
target_link_libraries(ekf ${catkin_LIBRARIES} )
target_link_libraries(imu_add_noise_quat ${catkin_LIBRARIES} )

catkin_install_python(PROGRAMS
    scripts/mun_frl_topic_bridge.py
    scripts/vo_gnss_imu_guidance.py
    scripts/vio_gnss_imu_guidance.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
