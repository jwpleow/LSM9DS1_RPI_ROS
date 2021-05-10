#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>
#include <mutex>

#include <memory>

#include "ros/ros.h"
#include "LSM9DS1_Types.h"
#include "LSM9DS1.h"


class IMU
{
public:
    LSM9DS1 imu_;

    // get ros timestamp and data in the original coord system of the imu (forward, right, up), and accel in G's, gyro in deg/s, and mag in gauss
    // note magnetometer polls at a slower rate than the rest, look at the settings in LSM9DS1.cpp
    // returns true if data was successfully read, false otherwise
    bool getAccelAndGyro(ros::Time &timestamp, float &gx, float &gy, float &gz, float &ax, float &ay, float &az)
    {
        if (imu_.gyroAvailable() && imu_.accelAvailable())
        {
            timestamp = ros::Time::now();
            imu_.readGyro();
            gx = imu_.calcGyro(imu_.gx); // rad/s
            gy = imu_.calcGyro(imu_.gy);
            gz = imu_.calcGyro(imu_.gz);
            imu_.readAccel();
            ax = imu_.calcAccel(imu_.ax);
            ay = imu_.calcAccel(imu_.ay);
            az = imu_.calcAccel(imu_.az);
            return true;
        }
        else
        {
            return false;
        }
    }

    bool getMag(ros::Time &timestamp, float &mx, float &my, float &mz)
    {
        if (imu_.magAvailable())
        {
            timestamp = ros::Time::now();
            imu_.readMag();
            mx = imu_.calcMag(imu_.mx);
            my = imu_.calcMag(imu_.my);
            mz = imu_.calcMag(imu_.mz);
            return true;
        }
        else
        {
            return false;
        }
    }

    IMU()
        : imu_(IMU_MODE_I2C, 0x6b, 0x1e)
    {
        imu_.begin();
        if (!imu_.begin())
        {
            fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
            exit(EXIT_FAILURE);
        }
        imu_.calibrate();
    }
    virtual ~IMU()
    {
    }
};