#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def callback(data):
    global Tau
    Tau = data.data
#Declare Variables to be used
k = 0.03
m = 0.75
l = 0.36 
g = 9.8
Tau = 0.0
x1 = 0.0
x2 = 0.0
lastTime = 0
# Setup Variables to be used
a = l/2
J = (4/3)*m*(a**2)


  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #Setup de subscribers
    sub = rospy.Subscriber('tau', Float32, callback)

    #Setup de publishers
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)


    joints = JointState()
    print("The SLM sim is Running")
    while not rospy.is_shutdown():
        try:

            #Run the node
            currentTime = rospy.get_time()
            dt = currentTime - lastTime
            if dt > 10:
                dt = 0

            x2_dot = (1/J)* (Tau-m*a*g*np.cos(x1)-k*x2)
            x1 += x2*dt
            x2 += x2_dot*dt
            print("Tau: ", Tau)

            joints.header.frame_id = "SLM"
            joints.header.stamp = rospy.Time.now()
            joints.name = ['joint2']
            joints.position = [wrap_to_Pi(x1)]
            joints.velocity = [x2]
            joints.effort = [Tau]
            pub.publish(joints)
            
            lastTime = currentTime


            loop_rate.sleep()
        
        except rospy.ROSInterruptException:
            pass #Initialise and Setup node