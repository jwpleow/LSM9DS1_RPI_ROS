# LSM9DS1 IMU on the Raspi, publishing to ROS
Replace the i2c bus your imu is connected to in `wiringPiI2CSetup` of `LSM9DS1_RaspberryPi_Library/include/wiringPiI2C.h`   

## Build
(i'm not bothering to use catkin here, so make sure you have sensor_msgs installed)
```
# source ros
mkdir build
cd build
cmake ..
```

## Run
Note: the library runs a small automatic calibration script when started, so ensure that the imu is level and stationary at the start
```
#source ros
./imu_ros
```
Publishes acceleration and gyro to `/imu/data_temp` and mag to `/imu/mag`.   

Note: the data needs to be calibrated for scales/axis misalignment!

### to publish to some other computer:
from <http://wiki.ros.org/ROS/NetworkSetup>   
replace the ip in ROS_MASTER_URI to the one running `roscore`   
e.g.
```
export ROS_MASTER_URI=http://192.168.1.58:11311/
export ROS_HOSTNAME=$(/bin/hostname) # these lower two should be the ip of the machine publishing
export ROS_IP=$(/bin/hostname -I) # this could return multiple ip's though
./imu_ros
```
Note: you may have to add the hostnames of each other in both computer's /etc/hosts   
you also might have to deactivate/allow connections in ufw 
