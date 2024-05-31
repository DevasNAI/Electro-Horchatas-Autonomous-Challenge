# Puzzlebot Running commands

FIRST RUN:
```
sudo chmod 666 /dev/ttyUSB1
roslaunch ros_deep_learning aruco_detector.ros1.launch
```

In another terminal:

```
roslaunch localisation puzzlebot_bug2.launch
```

If you want to diagnose the odometry in another terminal

```
rostopic echo /odom
```

