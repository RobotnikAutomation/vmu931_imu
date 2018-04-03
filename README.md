# imu_vmu931

ROS package to for the imu VMU931 from [Variense](https://variense.com/product/vmu931/).

## Params

* port (string)
  * default: /dev/ttyACM0
* frame_id (string)
  * default: imu_link
* mode (string)
  * device working mode. Defines which type of data is being streamed from the sensor.
  * Values:
    * quaternion-euler-heading (default)
    * gyro-accel-mag
    * custom: based on the params for each type of sensor
* gyroscope (bool)
  * Enables gyroscope streaming in custom mode
  * default: False
* magnetometer (bool)
  * Enables magnetometer streaming in custom mode
  * default: False
* accelerometer (bool)
  * Enables accelometer streaming in custom mode 
  * default: False
* quaternion (bool)
  * Enables quaternion streaming in custom mode 
  * default: True
* euler (bool)
  * Enables euler streaming in custom mode 
  * default: True
* heading (bool)
  * Enables heading streaming in custom mode 
  * default: True

 
## Topics
### Publishers

* ~data_raw (sensor_msgs/Imu)
  * Publishes the imu data raw from the sensor
  * Only available if quaternion values are being streamed by the sensor.
* ~quaternion (geometry_msgs/QuaternionStamped)
  * Publishes the quaternion received from the sensor
  * Only available if quaternion values are being streamed by the sensor.
* ~euler (geometry_msgs/Vector3Stamped)
  * Publishes the Euler angles received from the sensor
  * Only available if euler values are being streamed by the sensor.
* ~heading (geometry_msgs/Vector3Stamped)
  * Publishes the heading received from the sensor
  * Only available if heading values are being streamed by the sensor.
* ~gyro (geometry_msgs/Vector3Stamped)
  * Publishes the gyrocope values (rad/s) received from the sensor
  * Only available if gyro values are being streamed by the sensor.
* ~accelerometer (geometry_msgs/Vector3Stamped)
  * Publishes the accelerations received from the sensor
  * Only available if acceleration values are being streamed by the sensor.
* ~state (vmu931_imu/State)
  * Publishes the current state of the component

NOTE: output frequencies are different for every type of data

## Services
* ~/calibrate (std_srvs/Trigger)
  * Calibrates the sensor

  
## Bringup

```
roslaunch vmu931_imu vmu931_imu.launch
```


