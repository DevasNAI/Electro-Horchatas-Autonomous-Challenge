#!/usr/bin/env python3
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


KTheta = 7.5
KDistance = 0.5
trajectories = {
    "square": [(1, 0), (1, 1), (0, 1), (0, 0)],
    "pentagon": [(1, 0), (0.5, 0.87), (-0.5, 0.87), (-1, 0), (-0.5, -0.87)],
    "triangle": [(1, 0), (-0.5, 0.87), (-0.5, -0.87)]
}

trajectory = "triangle"

odom = Odometry()

def cb(odom_msg):
    global odom
    # Process the odometry message and generate the desired velocity comma
    odom = odom_msg

def wrapToPi(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi



if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('control', anonymous=True)

    rate = rospy.Rate(100)
    # Create a publisher for the /cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    goal = rospy.Publisher("/goal",PoseStamped,queue_size=10)
    dx = rospy.Publisher("/desired_x",Float32,queue_size=10)
    dy = rospy.Publisher("/desired_y",Float32,queue_size=10)
    dz = rospy.Publisher("/desired_z",Float32,queue_size=10)
    dw = rospy.Publisher("/desired_w",Float32,queue_size=10)

    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "odom"
    Dgoal_msg = Pose()

    # Create a subscriber to the /odom topic
    rospy.Subscriber('/odom', Odometry, cb)
    theta_set = False
    try:
        while not rospy.is_shutdown():
            for x,y in trajectories[trajectory]:
                print("Moving to: ", x, y)
                goal_msg.pose.position.x = x
                goal_msg.pose.position.y = y
                print("Current position: ", odom.pose.pose.position.x, odom.pose.pose.position.y)   
                theta = wrapToPi(tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])[2] )
                print("Current theta: ", theta)
                print("Desired theta ", math.atan2(y,x) - theta)
                pos = odom.pose.pose.position
                print("Error theta: ", wrapToPi(math.atan2(y - pos.y, x - pos.x) - theta))
                quat = tf.transformations.quaternion_from_euler(0, 0, math.atan2(y - pos.y, x - pos.x))

                while(True):
                    pos = odom.pose.pose.position
                    theta = wrapToPi(tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])[2])  
                    if (pos.x - x)**2 + (pos.y - y)**2 < 0.001:
                        break
                    cmd_vel = Twist()
                    errorx = x - pos.x
                    errory = y - pos.y

                    if theta_set == False:
                        errorTheta = abs(math.atan2(errory,errorx) - theta)
                        if abs(errorTheta) < 0.01:
                            theta_set = True
                        cmd_vel.angular.z = KTheta * errorTheta
                    else:
                        errorDistance = math.sqrt(errorx**2 + errory**2)
                        cmd_vel.linear.x = KDistance * errorDistance

                    # errorTheta = abs(math.atan2(errory,errorx) - theta)
                    # cmd_vel.angular.z = Kp * 2 * errorTheta    
                    # errorDistance = math.sqrt(errorx**2 + errory**2)
                    # cmd_vel.linear.x = Kp * errorDistance
                    
                    cmd_vel_pub.publish(cmd_vel)
                    goal.publish(goal_msg)
                    dx.publish(quat[0])
                    dy.publish(quat[1]) 
                    dz.publish(quat[2])
                    dw.publish(quat[3])
                    rate.sleep()
                theta_set = False
                rate.sleep()
            print("Trajectory completed")
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl-C caught. Exiting...")
        rospy.signal_shutdown("Ctrl-C caught")

