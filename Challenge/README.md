#  Robot Interfacing

This Challenge includes interfacing between a computer and a Puzzlebot Robot using gRPC, through a FLASK API, whose data is displayed on a Webpage.

#  Data Flow Diagram (DFD)
Here we can observe the DFD of the system we used

[localisation ROS Node] -> (/odom) -> [ROS-gRPC Wrapper] -> (odometryM) -> [gRPC-Client-FLASK] -> WebPage

## Odometry

| Type of Message | Odometry| X |
| ------------- | ------------- |  ------------- |
| Float32 | poseX | x |
| Float32 | poseX | x |
| Float32 | poseX | x |
| Float32 | orientationX | x |
| Float32 | orientationX | x |
| Float32 | orientationX | x |
| Float32 | orientationW | x |
| Float32 | orientationW | x |

## Mapping stack
| Type of Message | Map | X |
| ------------- | ------------- |  ------------- |
| x | x | x |
| x | x | x |

#  gRPC implementation
Install the requirements on the virtual environment.

`
pip install pythonvenv
pip install requirements.txt
`

Here is launch file

`
Command to run the launch file
`

# Webpage Visualization


