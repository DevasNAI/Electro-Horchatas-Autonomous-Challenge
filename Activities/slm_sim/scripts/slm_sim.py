#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Time
from sensor_msgs.msg import JointState

#Declare Variables to be used
k = 0.1
m = 0.75
l = 0.36
g = 9.8

x1 = 0.0
x2 = 0.0

# Declare the input Message
Tau = 0.0
# Declare the  process output message
Joints = JointState()

def joint_declaration(Joints):
    Joints.header.frame_id = "link1"
    Joints.header.stamp = rospy.Time.now()
    Joints.name.append("joint2")
    Joints.position.append(0.0)
    Joints.velocity.append(0.0)
    Joints.effort.append(0.0)

#wrap to pi function (COnverts position into angular position)
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

#Define the callback functions
def cbTau(msg):
    global Tau
    Tau = msg.data

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")
    joint_declaration(Joints)

    a = l/2
    # Setup Variables to be used
    J = (4/3)*(m * (np.power(a, 2)))

    #Get Parameters   
    previous_time = rospy.Time.now()

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))

    # Setup the Subscribers
    rospy.Subscriber("/tau", Float32, cbTau)

    #Setup de publishers
    statePub = rospy.Publisher("/joint_states", JointState, queue_size=1)
    #   x1 = posicion angular, x2 = velocidad angular, dt = tiempo
    posPub = rospy.Publisher("/angular_position", Float32, queue_size=1)
    velPub = rospy.Publisher("/angular_velocity", Float32, queue_size=1)
    timePub = rospy.Publisher("/angular_velocity", Time, queue_size=1)
    
    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)
            while not rospy.is_shutdown():

                now_time = rospy.Time.now()
                dt = (now_time - previous_time).to_sec()
                x1 += wrap_to_Pi(x2*dt)

                x2_dot = (1/(J+m*np.power(a,2))) * (-m*g*a* np.cos(x1)- k*x2 + Tau)#(1/J)*(Tau-(m*g*a*np.cos(x1) - (k*x2))) #(1/(J+(m*(np.power(a, 2))))) * (-(m*g*a*np.cos(x1) - (k*x2) + Tau))
                x2 += x2_dot * dt

                Joints.header.stamp = rospy.Time.now()
                Joints.position[0] = x1
                Joints.velocity[0] = x2


                statePub.publish(Joints)
                posPub.publish(x1)
                posPub.publish(x2)
                timePub.publish(dt)

                previous_time = now_time
                #Wait and repeat
                loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node