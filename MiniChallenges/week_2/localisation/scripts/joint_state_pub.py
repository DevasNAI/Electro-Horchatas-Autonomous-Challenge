#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

# Declare Variables to be used

## @var lastTime
#  Tiempo de la última actualización.
lastTime = 0.0

## @var dt
#  Diferencia de tiempo desde la última actualización.
dt = 0.0

## @var l
#  Distancia entre las ruedas del robot (en metros).
l = 0.19

## @var r
#  Radio de las ruedas del robot (en metros).
r = 0.05

## @var t
#  Tiempo actual.
t = 0.0

## @var x
#  Posición en el eje x del robot.
x = 0.0

## @var y
#  Posición en el eje y del robot.
y = 0.0

## @var w
#  Velocidad angular del robot.
w = 0.0

## @var z
#  Orientación del robot en el eje z.
z = 0.0

## @var vx
#  Velocidad lineal en el eje x del robot.
vx = 0.0

## @var vz
#  Velocidad angular en el eje z del robot.
vz = 0.0

## @var wlx
#  Posición de la rueda izquierda.
wlx = 0.0

## @var wrx
#  Posición de la rueda derecha.
wrx = 0.0

## @var odom
#  Mensaje de odometría recibido.
odom = Odometry()

# Setup Variables to be used

## @var joints
#  Estado de las articulaciones del robot.
joints = JointState()

# Declare the input Message

# Declare the  process output message

## Inicializa las articulaciones del robot.
def init_joints():
    joints.header.stamp = rospy.Time.now()
    joints.name.extend(["leftWheel", "rightWheel"])
    joints.position.extend([0.0, 0.0])
    joints.velocity.extend([0.0, 0.0])

#Define the callback functions

## Función de devolución de llamada para el mensaje de odometría.
#
#  Actualiza las variables globales con los datos de odometría recibidos.
#
#  @param data Mensaje de odometría.
def callback(data):
    global odom,vz,vx
    odom = data
    vz = odom.twist.twist.angular.z
    vx = odom.twist.twist.linear.x


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("puzzlebot_joint_state")
    loop_rate = rospy.Rate(100)

    tf_broadcaster = tf2_ros.TransformBroadcaster()  

    # Init Joints
    init_joints()

    # Setup the Subscribers
    rospy.Subscriber("/odom", Odometry, callback)

    #Setup de publishers
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    lastTime = rospy.Time.now().to_sec()
    
    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)
        while not rospy.is_shutdown(): 
             # Extract position and orientation from odometry message
            odom_trans = TransformStamped()
            odom_trans.header.stamp = rospy.Time.now()
            odom_trans.header.frame_id = "odom"
            odom_trans.child_frame_id = "base_link"
            odom_trans.transform.translation = odom.pose.pose.position
            odom_trans.transform.rotation = odom.pose.pose.orientation

            tf_broadcaster.sendTransform(odom_trans)

            # Set time and calculate dt
            t = rospy.Time.now().to_sec()
            dt = t - lastTime
            
            # Calculate wr y wl
            wr = ( ((vz * .19) / .05) + ( (2*vx) / .05 ) ) / 2
            wl = ( (2 * vx) / .05 ) - wr
            w = r * ( ( wr - wl ) / l )

            # Calculate the wheel position
            wrx += wr * dt
            wlx += wl * dt 

            # Set the information on the joints
            joints.header.stamp = rospy.Time.now()
            joints.position = [wlx, wrx]
            joints.velocity = [wl, wr]

            # Publish
            pub.publish(joints)

            # Save lastTime
            lastTime = t
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
