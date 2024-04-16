#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped

vel = Twist()
wl = Float32()
wr = Float32()
pose = PoseStamped()

l = 0.19
r = 0.05

def vel_cb(msg):
    global vel
    vel = msg
    rospy.loginfo("Linear velocity: %f, Angular velocity: %f", msg.linear.x, msg.angular.z)

def main():
    # Initialize the ROS node
    rospy.init_node('puzzelbot_kinematic_node', anonymous=True)

    rate = rospy.Rate(100) # 100 Hz

    # Subscriber
    rospy.Subscriber('cmd_vel', Twist, vel_cb)

    # Publisher
    pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
    wl_pub = rospy.Publisher('wl', Float32, queue_size=1)
    wr_pub = rospy.Publisher('wr', Float32, queue_size=1)

    prev_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        currentTime = rospy.Time.now().to_sec()

        dt = currentTime - prev_time
        prev_time = currentTime

        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x += vel.linear.x*dt*math.cos(pose.pose.orientation.z)
        pose.pose.position.y += vel.linear.x*dt*math.sin(pose.pose.orientation.z)
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z += vel.angular.z*dt
        pose.pose.orientation.z = pose.pose.orientation.z % (2*math.pi) # Normalize angle
        pose.pose.orientation.w = 0
        pose_pub.publish(pose)

        wl = ((vel.angular.z*r/l) + (2*vel.linear.x/r))/2
        wr = (2*vel.linear.x/r) - wl
        wl_pub.publish(wl)
        wr_pub.publish(wr)

        rate.sleep()

if __name__ == '__main__':
    main()