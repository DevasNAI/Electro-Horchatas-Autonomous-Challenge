#!/usr/bin/env python

#    A01245418 Andres Sarellano
#    Pendulum SLM_SIM Node, MCR Challenge 1

import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Time
from sensor_msgs.msg import JointState

#Declare Variables to be used
k = 0.1        
m = 0.75       #    Pendulum mass
l = 0.36       #    Lenght of joint to pendulum end effector
g = 9.8        #    Gravity

x1 = 0.0        #    Angular Position
x2 = 0.0        #    Angular Vlocity

#    Input signal
Tau = 0.0
#    Pendulum joint handler
Joints = JointState()

def joint_declaration(Joints):
    """
        Joints
        ------------
        @param Joints    sensors_msgs.JointState    |    JointState Handler

        Initialiazes a Joint.
    
    """
    #    Header of frame ID.
    Joints.header.frame_id = "link1"
    Joints.header.stamp = rospy.Time.now()
    #    Name of joint to be handled
    Joints.name.append("joint2")
    #    Initial state of position, velocity and effort. (Check ROS Documentation of this message).
    Joints.position.append(0.0)
    Joints.velocity.append(0.0)
    Joints.effort.append(0.0)

#wrap to pi function (COnverts position into angular position)
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

def cbTau(msg):
    """
        cbTau
        Tau Callback

        @param msg    |    Message to be calledback
    """
    global Tau
    #    Assings data from Topic to node's local variable.
    Tau = msg.data

if __name__=='__main__':
    #    Initialise and Setup node
    rospy.init_node("SLM_Sim")
    joint_declaration(Joints)
    #    Center of mass
    a = l/2
    #    Inertia momentum
    J = (4/3)*(m * (np.power(a, 2)))

    #    Initial Time for dt
    previous_time = rospy.Time.now()

    #    Configure the Node rate
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    #    Setup the Subscribers
    rospy.Subscriber("/tau", Float32, cbTau)

    #    Setup Publishers
    statePub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    #   x1 = Angular positoin, x2 = Angular velocity, dt = Time differential
    posPub = rospy.Publisher("/angular_position", Float32, queue_size=1)
    velPub = rospy.Publisher("/angular_velocity", Float32, queue_size=1)
    timePub = rospy.Publisher("/angular_velocity", Time, queue_size=1)
    
    print("The SLM sim is Running")
    try:
            while not rospy.is_shutdown():
                #    Gets runtime Time and Time differential
                now_time = rospy.Time.now()
                dt = (now_time - previous_time).to_sec()
                
                #    Calculates current position
                x1 += wrap_to_Pi(x2*dt)
                #    Calculates current velocity
                x2_dot = (1/(J+m*np.power(a,2))) * (-m*g*a* np.cos(x1)- k*x2 + Tau)#(1/J)*(Tau-(m*g*a*np.cos(x1) - (k*x2))) #(1/(J+(m*(np.power(a, 2))))) * (-(m*g*a*np.cos(x1) - (k*x2) + Tau))
                x2 += x2_dot * dt
                #    Updates header stamp for movement in rviz and the position and velocity of joint.
                Joints.header.stamp = rospy.Time.now()
                Joints.position[0] = x1
                Joints.velocity[0] = x2

                #    Publishes data into topics.
                statePub.publish(Joints)
                posPub.publish(x1)
                posPub.publish(x2)
                timePub.publish(dt)
                #    Updates time used this iteration as previous time for next iteration.
                previous_time = now_time
                #    Wait and repeat
                loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
