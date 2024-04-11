#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import numpy as np

#Declare Variables to be used
k = 0.015
m = 0.75
l = 0.36
g = 9.81
Tau = 0.0
x1 = 0.0
x2 = 0.0

#Define the callback functions
def tau_callback(data):
    global Tau
    Tau = data.data

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
    prev_Time = rospy.Time.now()
    a = l/2
    J = (4/3)*(m)*(a*a)

    # Configure the Node
    loop_rate = rospy.Rate(100)

    # Setup the Subscribers
    rospy.Subscriber("/tau", Float32, tau_callback)
    #Setup de publishers
    pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

    # Setup Variables to be used
    Joints = JointState()

    print("The SLM sim is Running")
    while not rospy.is_shutdown():
        #Run the node (YOUR CODE HERE)

        current_time= rospy.Time.now()
        rospy.loginfo(current_time)
        dt = (current_time - prev_Time).to_sec()
    
        x2_dot = (1/J) * (-m*g*a*np.cos(x1) - k*x2 + Tau)
        x1 += wrap_to_Pi(x2*dt)
        x2 += x2_dot * dt

        Joints.header.stamp = rospy.Time.now()
        Joints.header.frame_id = "SLM"
        Joints.name = ["joint2"]
        Joints.position = [x1]
        Joints.velocity = [x2]
        Joints.effort = [Tau]

        prev_Time=current_time

        pub.publish(Joints)
        
        loop_rate.sleep()
    