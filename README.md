# ICG ROS2 Wrapper
This is a ICG ROS2 wrapper repo of the [pose tracking library ICG](https://github.com/DLR-RM/3DObjectTracking/tree/master/ICG).


```
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=D455 serial_no:=_819612070593 rgb_camera.color_profile:=1920x1080x30
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=D455 serial_no:=_239722072823 rgb_camera.color_profile:=1280x720x30

```

```
colcon build --cmake-args -USE_REALSENSE=ON
```

```
ros2 launch ros2_aruco aruco_recognition.launch.py
```

```
ros2 launch icg_ros icg_test_node_launch.py
```

```
ros2 launch pose_subscriber pose_subscriber.launch.py
```


# Acknowledgments

Part of the code is borrowed from the [ICG ROS Wrapper](https://github.com/zhangbaozhe/icg_ros) repo.