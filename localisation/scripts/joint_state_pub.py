#!/usr/bin/env python
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

l = 0.19
r = 0.05
prev_time = 0
dt = 0
current_time = 0
odom = Odometry()

def odom_callback(msg):
    global odom
    odom = msg


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('joint_state_pub')

    # Create publishers for wl_joint and wr_joint topics
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    # Subscribe to the odom topic
    rospy.Subscriber('odom', Odometry, odom_callback)

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(100) # 100 Hz

    prev_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        # Extract position and orientation from odometry message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        odom_trans.transform.translation.x = odom.pose.pose.position.x
        odom_trans.transform.translation.y = odom.pose.pose.position.y
        odom_trans.transform.translation.z = odom.pose.pose.position.z
        odom_trans.transform.rotation = odom.pose.pose.orientation

        tf_broadcaster.sendTransform(odom_trans)
        # Calculate the time elapsed since the last callback
        current_time = rospy.Time.now().to_sec()
        dt = current_time - prev_time
        prev_time = current_time

        # Extract the odometry data
        linear = odom.twist.twist.linear.x
        angular = odom.twist.twist.angular.z

        # Calculate the wheel velocities
        wl = ((angular*r/l) + (2*linear/r))/2
        wr = (2*linear/r) - wl

        position_wr = 0
        position_wl = 0

        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['wl_joint', 'wr_joint']
        joint_state_msg.position = [position_wl, position_wr]
        joint_state_msg.velocity = [wl, wr]

        # Publish the JointState message
        joint_state_pub.publish(joint_state_msg)

        rate.sleep()