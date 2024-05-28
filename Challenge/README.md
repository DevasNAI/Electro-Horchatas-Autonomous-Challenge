#  Robot Interfacing

This Challenge includes interfacing between a computer and a Puzzlebot Robot using gRPC, through a FLASK API, whose data is displayed on a Webpage.

#  Data Flow Diagram (DFD)
Here we can observe the DFD of the system we used

<p align="center">
<img src="https://https://github.com/DevasNAI/Electro-Horchatas-Autonomous-Challenge/Images/DFD.png" width="50%" height="50%" title= "Data Flow Diagram" alt="DFD">
</p>

[localisation ROS Node] -> (/odom) -> [ROS-gRPC Wrapper] -> (odometryM) -> [gRPC-Client-FLASK] -> WebPage

## Odometry

| Type of Message | Odometry|
| ------------- | ------------- |
| Float32 | poseX |
| Float32 | poseY |
| Float32 | poseZ |
| Float32 | orientationX |
| Float32 | orientationY |
| Float32 | orientationZ |
| Float32 | orientationW |
| Double | LinXSpeed |
| Double | AngThetaSpeed |
| Float32 | orientationW |

## Mapping stack
| Type of Message | Map |
| ------------- | ------------- |
| byte | b64img |
| Float32 | width |
| Float32 | height |

#  gRPC implementation
Install the requirements on a virtual environment or locally on a computer with Python3 > 3.8

## Python dependencies
```
python -m pip3 install --upgrade pip
sudo python -m pip3 install grpcio
python -m pip3 install grpcio-tools
pip3 install Flask
pip3 install jsonlib
pip3 install schedule
pip3 install cv2
pip3 install numpy
```
## ROS
If you do not already have ROS installed, install ROS melodic (Ubuntu 18.04) or ROS Noetic (Ubuntu 20.04), preferibaly ROS Noetic.
Follow the documentation for your instalation on your respective OS.
https://wiki.ros.org/noetic/Installation/Ubuntu

# Implementation
## Puzzlebot
### Simulation (optional)
We have provided a simulation package for the puzzlebot if you want to run it locally before implementing on a Puzzlebot robot called _odom_flask_, if you run this package use the following commands:
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch odom_flask odom_flask.launch
```
### Real robot
If you have a Puzzlebot Lidar Jetson Edition under your possesion, probably the IP address for connecting to the hotspot is _10.42.0.1_, in our case we used ZeroTier, creating a VLAN (using ZeroTier) so that we can connect the puzzlebot to internet and access the robot from any place that has an internet connection registered on the puzzlebot. The IP address that was created to access our Puzzlebot is _10.242.43.39_, which should be used to connect to it through SSH and execute the commands below.

First, you must be sure that the hackerboard is sending the /cmd_vel topic, you may execute the command `rostopic list` inside the puzzlebot, if the topic does not show on the list or an error is marked, then run the command `sudo systemctl restart puzzlebot.service` so that the puzzlebot service that receives the messages from the Hackerboard is restarted.

After proving that the topics exist within the puzzlebot, just execute the following commands:

In one terminal run:
```
export ROS_IP=10.242.43.39
cd catkin_ws/src/localisation/scripts
python image_sender.py
```
In another terminal run:
```
export ROS_IP=10.242.43.39
roslaunch localisation puzzlebot_rviz.launch
```

If you have a different IP address where you may connect through SSH, you should change the export command to that IP Address, as well as on the gRPC server commands.


## gRPC Server and Client 
Once fully installed, on the Challenge-gRPC-python/ folder, execute the following commands on the terminal where you will run the gRPC server:
```
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://10.242.43.39:11311
export ROS_IP=<IP en ZeroTier, la de tu pc es 10.242.43.39>
python3 ros-grpc-server.py
```

Finally, in order to run the gRPC client, just execute the following command:
```
python3 grpc-client-flask.py
```

## Webpage Visualization
Once you have run all the commands, enter http://localhost:8000 to view the User Interface. If you run teleop_twist or configure the puzzlebot's launch file to run the controller to move to different points, then you will see the puzzlebot moving across the map.

