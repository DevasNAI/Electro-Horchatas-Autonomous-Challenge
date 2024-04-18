#!/usr/bin/env python3

# Imports
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import tf

## @file
#  @brief This file contains the implementation of a pose generator node.

## Define variables

## @var wl
#  Velocidad de la rueda izquierda.
wl = 0.0

## @var wr
#  Velocidad de la rueda derecha.
wr = 0.0

## @var vel_x
#  Velocidad lineal en la dirección x.
vel_x = 0.0

## @var ang_z
#  Velocidad angular alrededor del eje z.
ang_z = 0.0

## @var r
#  Radio de las ruedas.
r = 0.05

## @var l
#  Distancia entre las ruedas del robot.
l = 0.19

## @var v
#  Velocidad lineal.
v = 0.0

## @var va
#  Velocidad angular.
va = 0.0

## @var lastTime
#  Último tiempo registrado.
lastTime = 0.0

## @var dt
#  Diferencia de tiempo.
dt = 0.0

## Pose Variables

## @var angle
#  Ángulo actual de la pose.
angle = 0.0

## @var pos_x
#  Posición actual en x de la pose.
pos_x = 0.0

## @var pos_y
#  Posición actual en y de la pose.
pos_y = 0.0

## @var pose
#  Pose actual del robot.
pose = Pose()

## @var twist
#  Velocidades lineal y angular actuales del robot.
twist = Twist()

## @var odometry
#  Datos de odometría actuales del robot.
odometry = Odometry()

## Wrap angle to range [-pi, pi]
#
#  @param theta The angle to be wrapped.
#  @return The angle wrapped to the range [-pi, pi].
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

## Callback function for the WL subscriber.
#
#  Receives left wheel velocity data and updates the global variable wl.
#
#  @param msg The WL data.
def WL(msg):
    global wl
    wl = msg.data

## Callback function for the WR subscriber.
#
#  Receives right wheel velocity data and updates the global variable wr.
#
#  @param msg The WR data.
def WR(msg):
    global wr
    wr = msg.data

## Main function
if __name__ == "__main__":
    # Node Init
    rospy.init_node("pose_generator")
    loop_rate = rospy.Rate(100)

    # Publishers
    pub = rospy.Publisher("/odom", Odometry, queue_size=1)

    # Subscribers
    rospy.Subscriber("/wl", Float32, WL)
    rospy.Subscriber("/wr", Float32, WR)

    # Save current time
    lastTime = rospy.Time.now().to_sec()

    # Main loop
    while not rospy.is_shutdown():
        # Calculate time difference
        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - lastTime

        # Calculate linear speed
        v = r * ((wr + wl) / 2)

        # Calculate angular speed
        va = r * ((wr - wl) / l)

        # Calculate angle
        rospy.loginfo("dt: %f", dt)
        angle += va * dt
        angle = wrap_to_Pi(angle)

        # Calculate x and y
        vel_x = v * np.cos(angle)
        pos_x += vel_x * dt
        vel_y = v * np.sin(angle)
        pos_y += vel_y * dt

        # Set the Pose
        pose.position.x = pos_x
        pose.position.y = pos_y
        pose.position.z = 0.0

        # Transform from Euler to Quaternion
        rospy.loginfo("Angle: %f", angle)
    
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        # Set the Twist
        twist.linear.x = v
        twist.angular.z = va

        # Set the Odometry
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose = pose
        odometry.twist.twist = twist

        # Publish Odometry
        pub.publish(odometry)

        # Save lastTime
        lastTime = currentTime
        loop_rate.sleep()
