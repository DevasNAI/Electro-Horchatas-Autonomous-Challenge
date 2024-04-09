#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import math

#Declare Variables to be used
k = 0.01
m = 0.75
l = 0.36
g = 9.8
Tau = 0.0
x1 = 0.0
x2 = 0.0

Joints = JointState()

#Declare the output Messages
def init_joints():
    Joints.header.frame_id = "link1"
    Joints.header.stamp = rospy.Time.now()
    Joints.name.append("joint2")
    Joints.position.append(0.0)
    Joints.velocity.append(0.0)
    Joints.effort.append(0.0)

#Define the callback functions
def tau_callback(msg):
    global Tau
    Tau = msg.data

  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   
    prev_time = rospy.Time.now()
    a = l/2
    J = (4/3)*m*(a*a)

    # Configure the Node
    loop_rate = rospy.Rate(100)


    # Setup the Subscribers
    rospy.Subscriber('/tau', Float32, tau_callback)

    #Setup de publishers
    jointPub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    posPub = rospy.Publisher("/pos", Float32, queue_size=1)
    velPub = rospy.Publisher("/vel", Float32, queue_size=1)
    
    init_joints()

    print("The SLM sim is Running")
    #Run the node

    while not rospy.is_shutdown():
        #Wait and repeat
        current_time = rospy.Time.now()
        dt = (current_time - prev_time).to_sec()
        
        x1 += wrap_to_Pi(x2*dt)
        
        x2_dot = (1/(J+(m*a*a))) * (-m*g*a*math.cos(x1) - k*x2 + Tau)
        x2 += x2_dot*dt

        Joints.header.stamp = rospy.Time.now()
        Joints.position[0] = x1
        Joints.velocity[0] = x2
        
        jointPub.publish(Joints)
        posPub.publish(x1)
        velPub.publish(x2)

        prev_time = current_time
        loop_rate.sleep()
