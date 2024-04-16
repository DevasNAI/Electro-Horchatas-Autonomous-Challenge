import rospy
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf

#!/usr/bin/env python

l = 0.19
r = 0.05
x = 0.0
y = 0.0
theta = 0.0
wl = 0.0
wr = 0.0
def wr_cb(msg):
    global wr
    wr = msg.data
    rospy.loginfo("Right wheel velocity: %f", msg.data)
def wl_cb(msg):
    global wl
    wl = msg.data
    rospy.loginfo("Left wheel velocity: %f", msg.data)

def odometry_pub(wl, wr, dt):
    global x, y, theta
    # Calculate odometry based on wheel velocities
    # Replace the following lines with your own odometry calculation logic
    left_wheel_vel = wl
    right_wheel_vel = wr

    linear_vel = (left_wheel_vel + right_wheel_vel) / 2
    angular_vel = r*(right_wheel_vel - left_wheel_vel) / 0.19

    linear_vel_x = linear_vel * math.cos(theta) 
    linear_vel_y = linear_vel * math.sin(theta)
    theta += angular_vel

    x += linear_vel_x*dt
    y += linear_vel_y*dt
    x = 0.0
    y = 0.0
    theta += angular_vel*dt
    theta = theta % (2*math.pi) # Normalize angle

    # Create odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = 'base_link'
    odom_msg.child_frame_id = 'chassis'

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    odom_msg.pose.pose.orientation.x = quaternion[0]
    odom_msg.pose.pose.orientation.y = quaternion[1]
    odom_msg.pose.pose.orientation.z = quaternion[2]
    odom_msg.pose.pose.orientation.w = quaternion[3]

    odom_msg.twist.twist.linear.x = linear_vel
    odom_msg.twist.twist.angular.z = angular_vel

    # Publish odometry message
    odom_pub.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('localisation_node')

    # Subscribe to wheel velocity topics
    rospy.Subscriber('wl', Float32, wl_cb)
    rospy.Subscriber('wr', Float32, wr_cb)

    # Create odometry publisher
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

    rate = rospy.Rate(100) # 100 Hz
    prev_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        # Calculate time difference
        currentTime = rospy.Time.now().to_sec()
        dt = currentTime - prev_time
        prev_time = currentTime

        # Publish odometry message
        odometry_pub(wl, wr, dt)

        # Sleep to maintain the rate
        rate.sleep()