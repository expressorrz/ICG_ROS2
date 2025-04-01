# ICG ROS2 Wrapper
This is a ICG ROS2 wrapper repo of the [pose tracking library ICG](https://github.com/DLR-RM/3DObjectTracking/tree/master/ICG).


```
ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1 serial_no:=_819612070593
```

```
colcon build --packages-select icg_ros --cmake-args -USE_REALSENSE=ON
```

```
ros2 launch ros2_aruco aruco_recognition.launch.py
```

```
ros2 launch icg_ros icg_eyeinhand_launch.py

ros2 launch icg_ros icg_eyetohand_launch.py

```

```
ros2 launch pose_subscriber pose_subscriber.launch.py
```


# Acknowledgments

Part of the code is borrowed from the [ICG ROS Wrapper](https://github.com/zhangbaozhe/icg_ros) repo.