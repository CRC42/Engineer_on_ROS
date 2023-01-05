# sensor_ros_driver
## IMU
### 启动
1. 在src/test/initialization.cpp中检查使用IMU型号的波特率与代码是否对应  
2. ```sudo chown [user_name] /dev/ttyUSB0```
3. ``` roslaunch sensor_ros_driver wit.launch ```
### 直接发布在雷达坐标系
1. 先进行标定，得到imu->lidar的外参矩阵
2. 填入config/config.yaml中

## Camera
### 启动
1. ``` roslaunch sensor_ros_driver camera.launch ```
2. 相关参数设置在config/config.yaml
