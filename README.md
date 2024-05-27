# Simulation-Puzzlebot Challenge
This repository holds the progress and end results of the Puzzlebot Intelligent Robot Simulation Activities by Manchester Robotics for the TE30003B Integratino of Intelligent Robotics from Tecnológico de Monterrey.

## 1. Git clone this branch
## 2. Use the Docker Image (Intructions inside)
## 3. Put the packages in your workspace.
### Packages:
#### *Ours*
#### - aruco_detecter (Open camera, publish image_raw, detect arucos and estimate its position)
#### - localisation (Odometry, joint state publisher, controller and navigation)
#### *Manchester Robotics*
#### - puzzlebot_control (Low level control for puzzlebot simulation)
#### - puzzlebot_description (Spawn puzzlebot model on the simulation)
#### - puzzlebot_gazebo (Launch simulation, with different options of world, for different challenges)
### Change worlds:
#### Go to:
'''
/ws/src/puzzlebot_gazebo/launch/puzzlebot_gazebo.launch
'''
#### Modify the line 13, for any of the other options on the folder "/worlds".

## Enjoy!!!

## Authors

- [@DevasNAI](https://wwww.github.com/DevasNAI)
- [@JorgeAskur](https://www.github.com/JorgeAskur)
- [@PPMike](https://www.github.com/PPMike)
- [@Sebas677](https://www.github.com/Sebas677)
- [@Iz-Idk](https://www.github.com/Iz-Idk)

