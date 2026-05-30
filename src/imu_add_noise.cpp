#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <iostream>
#include <random>

using namespace std;
using namespace Eigen;

ros::Publisher imu_noise_pub;
double imu_noise = 0.05;
Vector3d noise;
std::default_random_engine generator;
std::normal_distribution<double> distribution_imu(0, imu_noise);

/// @brief Add zero-mean Gaussian noise to linear acceleration and republish IMU.
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    noise = Vector3d(distribution_imu(generator), distribution_imu(generator), distribution_imu(generator)); 

    sensor_msgs::Imu imu_noise;
    imu_noise = *msg;

    imu_noise.linear_acceleration.x += noise(0);
    imu_noise.linear_acceleration.y += noise(1);
    imu_noise.linear_acceleration.z += noise(2);

    imu_noise_pub.publish(imu_noise);
}

/// @brief ROS entry point for the standalone IMU acceleration-noise injector.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_add_noise_quat");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
    imu_noise_pub = n.advertise<sensor_msgs::Imu>("/djiros/imu_noise", 1000);

    n.getParam("imu_noise", imu_noise);
    distribution_imu = std::normal_distribution<double>(0.0, imu_noise);

    cout << "imu_noise: " << imu_noise << endl;

    ros::spin();
}
