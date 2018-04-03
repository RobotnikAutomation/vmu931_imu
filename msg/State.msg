robotnik_msgs/State state
bool gyroscope_status
bool accelerometer_status
bool magnetometer_status
# flag active during the calibration
bool calibrating
# working mode in terms of the type of data is being streamed
# quaternion-euler-heading, gyro-accel-mag, custom
string mode
# sensor output rate
int32 output_rate
# flags to show what info is being streamed by the sensor
bool quaternion_streaming
bool euler_streaming
bool heading_streaming
bool gyro_streaming
bool accel_streaming
bool magnet_streaming

