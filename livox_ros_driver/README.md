# 激光雷达驱动程序
## 依赖
1. 在安装livox_ros_driver之前，必须先编译好sensor_ros_driver
2. 其余依赖安装见[Livox-SDK](https://github.com/Livox-SDK/livox_ros_driver/)
## 安装
1. 在根据Livox-SDK步骤安装完后，把本livox_ros_driver替换Livox-SDK安装的livox_ros_driver
2. livox_ros_driver改动
*   livox_ros_driver/lddc.cpp:支持外置IMU(搜索R2LIVE可以看到)
*   livox_ros_driver/livox_ros_driver.cpp:对相机进行软同步(搜索CameraNode可以看到)
## 编译
```
catkin_make -DSENSOR_ROS_DRIVER_DIRS:STRING=[sensor_ros_driver的根目录]
```
例如
```
catkin_make -DSENSOR_ROS_DRIVER_DIRS:STRING=/home/liwei/catkin_ws/src/sensor_ros_driver
```
## 启动
同时启动相机和激光雷达
```
roslaunch livox_ros_driver livox_lidar_msg.launch
```