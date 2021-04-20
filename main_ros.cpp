#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include <memory>
#include <chrono>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "imu.h"

int main(int argc, char **argv)
{
    IMU imu;

    ros::init(argc, argv, "imu_publisher");

    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 50);
    ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 50);

    ros::Rate rate(70.0); // mag maxes out at 80Hz, perhaps poll data separately?

    uint32_t seq = 0;
    if (ros::ok())
        ROS_INFO("IMU publisher started");
    ros::Time timestamp;
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    while (ros::ok())
    {
        seq++;
        sensor_msgs::Imu imu_msg;
        sensor_msgs::MagneticField mag_msg;
        while (!imu.getData(timestamp, gx, gy, gz, ax, ay, az, mx, my, mz)) continue;

        imu_msg.header.seq = seq;
        mag_msg.header.seq = seq;
        imu_msg.header.stamp = timestamp;
        mag_msg.header.stamp = timestamp;

        imu_msg.header.frame_id = "imu_link";
        mag_msg.header.frame_id = "imu_link";

        imu_msg.angular_velocity.x = gx * M_PI / 180.0; // deg/s -> rad/s // forward
        imu_msg.angular_velocity.y = - gy * M_PI / 180.0; // left
        imu_msg.angular_velocity.z = gz * M_PI / 180.0; // up

        imu_msg.linear_acceleration.x = ax * 9.81; // g's -> m/s^2
        imu_msg.linear_acceleration.y = - ay * 9.81; // +ve for nose down
        imu_msg.linear_acceleration.z = az * 9.81;

        mag_msg.magnetic_field.x = mx / 10000.0; // gauss -> tesla
        mag_msg.magnetic_field.y = my / 10000.0;
        mag_msg.magnetic_field.z = mz / 10000.0;

        // no orientation
        imu_msg.orientation_covariance[0] = -1;

        imu_pub.publish(imu_msg);
        mag_pub.publish(mag_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}