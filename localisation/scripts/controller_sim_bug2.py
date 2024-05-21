#!/usr/bin/env python3
import rospy
import math
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import enum

# Define the state machine
class StateMachine(enum.Enum):
    LOOK_TOGOAL = 1
    FOLLOW_LINE = 2
    WALL_FOLLOW = 3
    STOP = 4

yaw = 0.0

# Initialize the state machine
current_state = StateMachine.LOOK_TOGOAL

# Initialize the current pose
current_pose = PoseStamped()
current_pose.header.frame_id = "world"
current_pose.pose.position.x = 2
current_pose.pose.position.y = 2

# Initialize the start pose
start_pose = PoseStamped()
start_pose.header.frame_id = "world"
start_pose.pose.position.x = 2
start_pose.pose.position.y = 2

# Initialize the goal pose
goal = PoseStamped()
goal.header.frame_id = "world"
# Set the goal position
goal.pose.position.x = 0
goal.pose.position.y = -2.5

cmd_vel = Twist()

# Initialize the hitpoint
hitpoint = None

# Initialize the distance moved
distance_moved = 0.0

# Initialize the min distance to the wall from the robot per 30 deg segment (in meters)
front_distance = 0.0
right_distance = 0.0
frontR_distance = 0.0
frontL_distance = 0.0
left_distance = 0.0

cmd_vel_pub = None

# Initialize the line slope for calculating the distance to the line
line_slope_m = (
                (goal.pose.position.y - start_pose.pose.position.y) / (
                goal.pose.position.x - start_pose.pose.position.x))

# Calculate the y-intercept point (point where the line crosses the y-axis)
# y = mx + b
# b = y - mx    
line_slope_b = start_pose.pose.position.y - (line_slope_m * start_pose.pose.position.x)

def wrapToPi(angle):
    if np.fabs(angle) > np.pi:
        angle = angle - (2*np.pi*angle) / (np.fabs(angle))
    return angle

def look_to_goal():
    global yaw, current_pose, goal, distance_moved, current_state, cmd_vel, cmd_vel_pub

    # Current orientation of the robot
    quaternion = (current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]

    # Calculate the angle to the goal
    angle_to_goal = math.atan2(goal.pose.position.y - current_pose.pose.position.y, goal.pose.position.x - current_pose.pose.position.x)
    angle_error = wrapToPi(angle_to_goal - yaw)

    # Rotate the robot towards the goal ( 1 radian error threshold )
    if np.fabs(angle_error) > np.pi/90:
        cmd_vel.angular.z = 0.5 if angle_error > 0 else -0.5
    else:
        # Move the robot towards the goal
        cmd_vel.angular.z = 0.0
        current_state = StateMachine.FOLLOW_LINE

    # Publish the velocity command
    cmd_vel_pub.publish(cmd_vel)

def move_to_goal():
    global current_state, current_pose, hitpoint

    # Check if the robot is close to the wall
    if np.any((front_distance < 0.3)):
        # Stop the robot
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0

        # Set the hitpoint to the current pose
        hitpoint = current_pose.pose.position

        current_state = StateMachine.WALL_FOLLOW
    else:
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 0.0

    # Publish the velocity command
    cmd_vel_pub.publish(cmd_vel)

def follow_wall():
    global cmd_vel, cmd_vel_pub, current_state, hitpoint, current_pose, front_distance, right_distance, frontR_distance, frontL_distance, left_distance, distance_moved, line_slope_b, line_slope_m

    # Get the closest point to the goal line
    closestGoalLine_x = current_pose.pose.position.x
    closestGoalLine_y = line_slope_m*(current_pose.pose.position.x) + line_slope_b

    # Calculate the distance moved by the robot
    distance_moved = math.sqrt((current_pose.pose.position.x - hitpoint.x)**2 + (current_pose.pose.position.y - hitpoint.y)**2)
    # Get the distance to the goal line
    distance_to_line = math.sqrt((closestGoalLine_x - current_pose.pose.position.x)**2 + (closestGoalLine_y - current_pose.pose.position.y)**2)

    # Check if the robot is close to the goal line and has moved a certain distance
    if distance_to_line < 0.1 and distance_moved > 0.5:
        # Calculate the distance to the goal
        distance_to_goal = math.sqrt((goal.pose.position.x - current_pose.pose.position.x)**2 + (goal.pose.position.y - current_pose.pose.position.y)**2)
        # Calculate the distance to the hitpoint
        hitpoint_distance_to_goal = math.sqrt((goal.pose.position.x - hitpoint.x)**2 + (goal.pose.position.y - hitpoint.y)**2)

        # Check if the distance to the hitpoint is greater than the distance to the goal
        if hitpoint_distance_to_goal > distance_to_goal:
            # Stop the robot
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            # Set the current state to look to goal
            current_state = StateMachine.LOOK_TOGOAL
            return
    # Adjust the robot's orientation to be parallel to the wall
    elif np.any((front_distance < 0.3)):
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = -0.5
    # Adjust the robot's orientation to keep following the wall
    elif np.any((frontL_distance >= 0.2)):
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.5
    else:
        # Move the robot forward
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 0.0
    
    # Publish the velocity command
    cmd_vel_pub.publish(cmd_vel)

def goal_callback(msg):
    global goal, start_pose, line_slope_m, line_slope_b, current_pose, current_state
    print("New goal received!")
    
    # Update the goal pose
    goal = msg
    # Update the start pose
    start_pose = current_pose
    print("Goal: ", goal.pose.position.x, goal.pose.position.y)

    # Calculate the slope of the line connecting the start and goal points
    line_slope_m = (
                (goal.pose.position.y - start_pose.pose.position.y) / (
                goal.pose.position.x - start_pose.pose.position.x))
    # Calculate the y-intercept point (point where the line crosses the y-axis)     
    line_slope_b = start_pose.pose.position.y - (line_slope_m * start_pose.pose.position.x)
    # Set the current state to look to goal
    current_state = StateMachine.LOOK_TOGOAL
    

def odom_callback(msg):
    global current_pose
    # Update the current pose
    current_pose.pose = msg.pose.pose
    
def scan_callback(msg):
    global right_distance, frontR_distance, front_distance, frontL_distance, left_distance

    # Get the minimum distance from the laser scan
    data = np.array(msg.ranges)

    # Get the minimum distance for each segment 30 degrees
    right_distance = np.min(data[0:50])
    frontR_distance = np.min(data[51:140])
    front_distance = np.min(data[141:220])
    frontL_distance = np.min(data[221:310])
    left_distance = np.min(data[311:361])

def stop():
    global cmd_vel_pub
    # Stop the robot
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)

def main():
    global current_pose, current_state, cmd_vel, cmd_vel_pub

    print("Starting controller node")
    rospy.init_node('controller_sim_bug2')

    # Create publishers and subscribers
    cmd_vel_pub = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber('/puzzlebot_1/base_pose_ground_truth', Odometry, odom_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    rospy.Subscriber('/puzzlebot_1/scan', LaserScan, scan_callback)

    rospy.on_shutdown(stop)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():        
        # Check the current state
        if current_state is StateMachine.LOOK_TOGOAL:
            look_to_goal()
        elif current_state is StateMachine.FOLLOW_LINE:
            move_to_goal()
        elif current_state is StateMachine.WALL_FOLLOW:
            follow_wall()
        elif current_state is StateMachine.STOP:
            # Stop the robot
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            print("Found goal!")
        # Publish the velocity command
        cmd_vel_pub.publish(cmd_vel)
        # Calculate the distance to the goal
        goal_distance = math.sqrt((goal.pose.position.x - current_pose.pose.position.x)**2 + (goal.pose.position.y - current_pose.pose.position.y)**2)

        # Check if the robot has reached the goal
        if goal_distance < 0.15:
            current_state = StateMachine.STOP

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass