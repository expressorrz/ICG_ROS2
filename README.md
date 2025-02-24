# ICG ROS2 Wrapper
This is a ICG ROS2 wrapper repo of the [pose tracking library ICG](https://github.com/DLR-RM/3DObjectTracking/tree/master/ICG).


```
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=D455 serial_no:=_239722072823
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


data: [-0.7750956190862293, 0.5750350841341046, -0.26184238252853137, 0.19079416005256802,
0.5019919651297154, 0.30877371964185574, -0.8078755207355518, 0.11421521520477351,
-0.3837067216228314, -0.7576235490488766, -0.5279921492863722, 0.6390919710009377,
0.0, 0.0, 0.0, 1.0]

data: [-0.7750956190862293, 0.5750350841341046, -0.26184238252853137, -0.19079416005256802,
          0.5019919651297154, 0.30877371964185574, -0.8078755207355518, 0.11421521520477351,
          -0.3837067216228314, -0.7576235490488766, -0.5279921492863722, 0.6390919710009377,
          0, 0, 0, 1 ]

data: [0.607674, 0.786584, -0.10962, -0.081876,
          0.408914, -0.428214, -0.805868, -0.00546736,
          -0.680823, 0.444881, -0.58186, 0.618302,
          0, 0, 0, 1 ]


# Acknowledgments

Part of the code is borrowed from the [ICG ROS Wrapper](https://github.com/zhangbaozhe/icg_ros) repo.