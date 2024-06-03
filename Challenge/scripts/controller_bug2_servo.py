#!/usr/bin/env python
import rospy
import math
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from ros_deep_learning.msg import ArucosDetected, ArucoDetected
import enum

# Define the state machine for the Bug2 algorithm
class StateMachine(enum.Enum):
    LOOK_TOGOAL = 1
    FOLLOW_LINE = 2
    WALL_FOLLOW = 3
    STOP = 4
    VISUAL_SERVO = 5
    VISUAL_BASE = 6
    REVERSE = 7

class Bug2Controller:
    def __init__(self):
        # Initialize various parameters and ROS node
        self.yaw = 0.0
        self.current_state = StateMachine.LOOK_TOGOAL

        # Initialize current pose of the robot
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "odom"
        self.current_pose.pose.position.x = 0
        self.current_pose.pose.position.y = 0

        # Initialize starting pose of the robot
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = "odom"
        self.start_pose.pose.position.x = 0
        self.start_pose.pose.position.y = 0

        # Set the goal position for the robot
        self.goal = PoseStamped()
        self.goal.header.frame_id = "odom"
        self.goal.pose.position.x = 2.5
        self.goal.pose.position.y = 2.5

        #   Different Goal coordinates
        #self.goal_changer = ["cube", "base_a", "base_b", "base_c"]
        #self.feedback = rospy.get_param("/new_pose", "")

        self.cmd_vel = Twist()  # Velocity command
        self.hitpoint = None  # Point where the robot hits an obstacle
        self.distance_moved = 0.0  # Distance moved by the robot

        # Distances to the nearest obstacles in different directions
        self.front_distance = 0.0
        self.frontL_distance = 0.0
        self.left_distance = 0.0

        self.past_front_distance = 0.0
        self.future_distance = 0.0

        # Servo variables marker = 0.143 cube = 0.05
        self.aruco = ArucoDetected()
        self.aruco_x = 0.0
        self.aruco_y = 0.0

        self.aruco_base = ArucoDetected()
        self.aruco_base_x = 0.0
        self.aruco_base_y = 0.0

        self.maxSpeed_base = 0.025
        self.p_base = 0.6
        self.d_base = 0.1

        self.maxSpeed_cube = 0.0175
        self.p_cube = 0.6
        self.d_cube = 0.2

        self.prev_error = 0.0
        self.prev_base_error = 0.0

        self.desired_pos = 0.0175

        self.last_time = 0.0

        self.avgError = []
        self.avg_baseError = []

        self.reverse_start_time = None


        self.base_flag = False #BASE APPROACH
        self.close_val = 116
        self.open_val = 0
        self.cube = False

        # Initialize ROS publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.servo_pub = rospy.Publisher('/servo', Float32, queue_size=1)
        #self.goal_ack_pub = rospy.Publisher('/goal_ack', String, queue_size=1)

        # Initialize ROS subscribers for odometry, goal, and laser scan data
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/aruco_detected', ArucoDetected, self.aruco_base_callback)
        rospy.Subscriber('/cube_detected', ArucoDetected, self.aruco_callback)

        # Calculate line parameters (slope and y-intercept) from start to goal
        self.line_slope_m = (self.goal.pose.position.y - self.start_pose.pose.position.y) / (
                self.goal.pose.position.x - self.start_pose.pose.position.x)
        self.line_slope_b = self.start_pose.pose.position.y - (self.line_slope_m * self.start_pose.pose.position.x)

        # Set up shutdown behavior
        rospy.on_shutdown(self.stop)
        self.rate = rospy.Rate(5)  # Loop rate in Hz

    def wrap_to_pi(self, angle):
        # Wrap an angle to the range [-pi, pi]
        if np.fabs(angle) > np.pi:
            angle = angle - (2*np.pi*angle) / (np.fabs(angle))
        return angle

    def look_to_goal(self):
        # Orient the robot towards the goal
        quaternion = (self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]  # Get the current yaw of the robot

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(self.goal.pose.position.y - self.current_pose.pose.position.y, self.goal.pose.position.x - self.current_pose.pose.position.x)
        angle_error = angle_to_goal - self.yaw
        print("Angle error ", angle_error)
        print("Angle to goal ", angle_to_goal)
        # Rotate the robot towards the goal if the angle error is significant
        if np.fabs(angle_error) > np.pi/90:
            self.cmd_vel.angular.z = 0.25 if angle_error > 0 else -0.25
        else:
            self.cmd_vel.angular.z = 0.0
            self.current_state = StateMachine.FOLLOW_LINE  # Switch to FOLLOW_LINE state
            #print(self.current_state)

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def move_to_goal(self):
        # Move the robot towards the goal
        if np.any((self.front_distance < 0.3)) :  # Stop if an obstacle is detected in front
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.hitpoint = self.current_pose.pose.position  # Record the hitpoint
            self.current_state = StateMachine.WALL_FOLLOW  # Switch to WALL_FOLLOW state
            #print(self.current_state)
        else:
            distance_to_goal = math.sqrt((self.goal.pose.position.x - self.current_pose.pose.position.x)**2 + (self.goal.pose.position.y - self.current_pose.pose.position.y)**2)
            if self.aruco != None and 0 < self.aruco_x < 0.5 and self.cube == False:
                self.current_state = StateMachine.VISUAL_SERVO  # Switch to APPROACH_CUBE state
        
            elif self.aruco_base != None and 0 < self.aruco_base_x < 0.6 and self.base_flag == False:
                self.current_state = StateMachine.VISUAL_BASE   #Switch to APPROACH_BASE state

            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def follow_wall(self):
        # Follow the wall until a certain condition is met
        closestGoalLine_x = self.current_pose.pose.position.x
        closestGoalLine_y = self.line_slope_m * self.current_pose.pose.position.x + self.line_slope_b

        self.distance_moved = math.sqrt((self.current_pose.pose.position.x - self.hitpoint.x)**2 + (self.current_pose.pose.position.y - self.hitpoint.y)**2)
        distance_to_line = math.sqrt((closestGoalLine_x - self.current_pose.pose.position.x)**2 + (closestGoalLine_y - self.current_pose.pose.position.y)**2)

        if self.aruco_base != None and 0 < self.aruco_base_x < 0.6 and self.base_flag == False:
            self.current_state = StateMachine.VISUAL_BASE #***same MOD as MOVE_TO_GOAL

	    #print("Distance to line: ", distance_to_line)
        if distance_to_line < 0.1 and self.distance_moved > 0.5:
            distance_to_goal = math.sqrt((self.goal.pose.position.x - self.current_pose.pose.position.x)**2 + (self.goal.pose.position.y - self.current_pose.pose.position.y)**2)
            #print("Distance to goal: ", distance_to_goal)
            hitpoint_distance_to_goal = math.sqrt((self.goal.pose.position.x - self.hitpoint.x)**2 + (self.goal.pose.position.y - self.hitpoint.y)**2)
            print(self.aruco_base_x)
            if self.aruco != None and 0 < self.aruco_x < 0.5 and self.cube == False:
                self.current_state = StateMachine.VISUAL_SERVO #****MOD_SEB 

            elif hitpoint_distance_to_goal > distance_to_goal:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.current_state = StateMachine.LOOK_TOGOAL  # Switch to LOOK_TOGOAL state
                #print(self.current_state)
                return
        elif np.any((self.front_distance < 0.4)):
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = -0.5
        elif np.any((self.frontL_distance >= 0.35)):
            self.cmd_vel.linear.x = 0.15
            self.cmd_vel.angular.z = 0.5
        else:
            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command

    def goal_callback(self, msg):
        # Update the goal when a new goal message is received
        self.goal = msg
        #print(self.goal)
        
        self.start_pose = self.current_pose

        self.line_slope_m = (self.goal.pose.position.y - self.start_pose.pose.position.y) / (
                            self.goal.pose.position.x - self.start_pose.pose.position.x)
        self.line_slope_b = self.start_pose.pose.position.y - (self.line_slope_m * self.start_pose.pose.position.x)
        self.current_state = StateMachine.LOOK_TOGOAL  # Switch to LOOK_TOGOAL state

        self.cube = True

    def odom_callback(self, msg):
        # Update the current pose based on odometry data
        self.current_pose.pose = msg.pose.pose

    def scan_callback(self, msg):
        # Update the distances to obstacles based on laser scan data
        data = np.array(msg.ranges)
        self.deltaT = msg.scan_time
        self.past_front_distance = self.front_distance
        dummy = np.array(data[0:90])
        np.append(dummy, np.array(data[1015:]))

        self.front_distance = np.min(dummy)
        self.future_distance = ((self.front_distance - self.past_front_distance) / self.deltaT) + self.front_distance
        self.frontL_distance = np.min(data[91:180])

    def aruco_callback(self, msg):
        self.aruco = msg
        self.aruco_x = self.aruco.pose.position.x
        self.aruco_y = self.aruco.pose.position.y

    def aruco_base_callback(self,msg):
        self.aruco_base = msg
        self.aruco_base_x = self.aruco_base.pose.position.x
        self.aruco_base_y = self.aruco_base.pose.position.y
        print("Base arucos x: " ,self.aruco_base_x)

    def visual_base(self):

        #HANDLE RECEIVED LIST 

        error = self.desired_pos - self.aruco_base_y
        #print("Distancia al cubo: ", self.aruco_x)
        avg = 0
        current_time = rospy.get_time()
        dt = current_time - self.last_time

        pd = self.p_base*error + self.d_base*(error - self.prev_base_error)/dt

        self.prev_base_error = error
        self.last_time = current_time
        speed = round(float(pd),2)

        if len(self.avg_baseError)>=10:
                self.avg_baseError.pop()
        self.avg_baseError.append(error)

        for error in self.avg_baseError:
            avg+= error

        avg = avg/len(self.avg_baseError)

        vel = abs(self.maxSpeed_base-(abs(float(avg))/1000))
        line=round(vel,3)

        cmd_vel = Twist()
        cmd_vel.linear.x = line
        cmd_vel.angular.z = speed
        if self.aruco_base_x <= 0.20:
            cmd_vel.linear.x = 0
            #self.current_state = StateMachine.STOP
            self.cmd_vel_pub.publish(cmd_vel)
            self.servo_pub.publish(self.open_val)
            self.current_state = StateMachine.REVERSE
        else:
            self.cmd_vel_pub.publish(cmd_vel)


    def visual_servo(self):
        error = self.desired_pos - self.aruco_y
        #print("Distancia al cubo: ", self.aruco_x)
        avg = 0
        current_time = rospy.get_time()
        dt = current_time - self.last_time

        pd = self.p_cube*error + self.d_cube*(error - self.prev_error)/dt

        self.prev_error = error
        self.last_time = current_time
        speed = round(float(pd),2)

        if len(self.avgError)>=10:
            self.avgError.pop()
        self.avgError.append(error)

        for error in self.avgError:
            avg+= error

        avg = avg/len(self.avgError)

        vel = abs(self.maxSpeed_cube-(abs(float(avg))/1000))
        line=round(vel,3)

        cmd_vel = Twist()
        cmd_vel.linear.x = line
        cmd_vel.angular.z = speed
        if self.aruco_x <= 0.075:
            cmd_vel.linear.x = -8
            self.cmd_vel_pub.publish(cmd_vel)
            self.cmd_vel_pub.publish(cmd_vel)
            self.cmd_vel_pub.publish(cmd_vel)
            self.cmd_vel_pub.publish(cmd_vel)
            cmd_vel.linear.x = 0
            self.cmd_vel_pub.publish(cmd_vel)
            print("Cerrando servo")
            self.servo_pub.publish(self.close_val)
            self.current_state = StateMachine.STOP
        else:
            self.servo_pub.publish(0)
            self.cmd_vel_pub.publish(cmd_vel)

    def stop(self):
        # Stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        #if self.cube == True:
            #self.servo_pub.publish(0)
            #print("Aruco entregado")

        #se chinga el system ___seb
        #self.feedback = "origin"

    def reverse(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.2
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        if self.reverse_start_time is None:
            self.reverse_start_time = rospy.get_time()

        if rospy.get_time() - self.reverse_start_time > 0.8:
            self.current_state = StateMachine.STOP
            self.base_flag = True
            self.reverse_start_time = None




    def run(self):
       # self.goal_ack_pub.publish(self.goal_changer[0])
        # Main loop to control the robot
        while True:
            if self.current_state is StateMachine.VISUAL_BASE:
                self.visual_base()
            elif self.current_state is StateMachine.VISUAL_SERVO:
                self.visual_servo()
            elif self.current_state is StateMachine.LOOK_TOGOAL:
                self.look_to_goal()
            elif self.current_state is StateMachine.FOLLOW_LINE:
                self.move_to_goal()
            elif self.current_state is StateMachine.WALL_FOLLOW:
                self.follow_wall()
            elif self.current_state is StateMachine.STOP:
                self.stop()
            elif self.current_state is StateMachine.REVERSE:
                self.reverse()
            print(self.current_state)
            
            #self.cmd_vel_pub.publish(self.cmd_vel)  # Publish the velocity command
            
            goal_distance = math.sqrt((self.goal.pose.position.x - self.current_pose.pose.position.x)**2 + (self.goal.pose.position.y - self.current_pose.pose.position.y)**2)
            print("goal distance: ", goal_distance)
            deltax = self.goal.pose.position.x - self.current_pose.pose.position.x
            deltay = self.goal.pose.position.y - self.current_pose.pose.position.y


            #if deltax < 0.05 and deltay < 0.05 and self.current_state != StateMachine.STOP and self.base_flag ==True:  # Stop if the goal $
                #self.current_state = StateMachine.STOP 
                #self.goal_changer[newGoaliD]
                #print("Found Goal!")

            ##YA NO SE USA____SEB___
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('controller_node')
        controller = Bug2Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass

