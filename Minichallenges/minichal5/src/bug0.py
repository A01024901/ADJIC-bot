#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan  # Lidar
import numpy as np

# This class directs the puzzlebot towards a specified goal
class NavigateToGoal():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        ############ ROBOT PARAMETERS ############

        self.r = 0.05  # Wheel radius
        self.L = 0.19  # Wheel separation

        ############ INITIAL CONDITIONS ############

        self.x = 0.0  # Initial x position of the robot 
        self.y = 0.0  # Initial y position of the robot 
        self.theta = 0.0  # Initial orientation of the robot

        ############ GOAL VARIABLES ############

        self.x_target = 0.0  # x-coordinate of the goal
        self.y_target = 0.0  # y-coordinate of the goal
        self.goal_received = False  # Flag goal received
        self.lidar_received = False  # Flag laser scan received
        self.target_position_tolerance = 0.001  # Tolerance goal reached
        self.integral = 0.0
        self.prev_error = 0.0
        fw_distance = 0.30  # Distance to switch to wall-following mode
        stop_distance = 0.001  # Distance to stop the robot if an obstacle is close 
        eps = 0.20  # Safety buffer
        v_msg = Twist() 
        self.wr = 0  # Right wheel speed [rad/s]
        self.wl = 0  # Left wheel speed [rad/s]
        self.current_state = 'NavigateToGoal'  

        ### INITIALIZE PUBLISHERS ###

        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)

        ####################### SUBSCRIBERS #######################

        rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)
        rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)
        rospy.Subscriber("puzzlebot_goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)

        ### INITIALIZE NODE ###
        freq = 20
        rate = rospy.Rate(freq)  
        dt = 1.0 / float(freq)  
        fwf = True
        d_t1 = 0.0
        d_t = 0.0
        theta_ao = 0.0
        theta_gtg = 0.0

        ############### MAIN LOOP ###############
        while not rospy.is_shutdown():
            self.update_state(self.wr, self.wl, dt)  # Update the robot's state

            if self.lidar_received:
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg)  # Get closest obstacle

                if self.current_state == 'Stop':
                    if self.goal_received:
                        print("Transitioning from Stop to NavigateToGoal")
                        self.current_state = "NavigateToGoal"
                        self.goal_received = False

                    else:
                        v_msg.linear.x = 0.0
                        v_msg.angular.z = 0.0

                elif self.current_state == 'NavigateToGoal':
                    if self.at_goal() or closest_range < stop_distance:
                        print(f"Transitioning to Stop from NavigateToGoal. Target: ({self.x_target}, {self.y_target})")
                        self.current_state = "Stop"

                    elif closest_range < fw_distance:
                        print("Transitioning to WallFollower from NavigateToGoal")
                        self.current_state = "WallFollower"

                    else:
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.x, self.y, self.theta)
                        v_msg.linear.x = v_gtg
                        v_msg.angular.z = w_gtg

                elif self.current_state == 'WallFollower':
                    theta_gtg, theta_ao = self.compute_angles(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle)
                    d_t = np.sqrt((self.x_target - self.x)**2 + (self.y_target - self.y)**2)
                    print(f"Evaluating transition from WallFollower. Distance to target: {d_t}")

                    if self.at_goal() or closest_range < stop_distance:
                        print("Transitioning to Stop")
                        self.current_state = "Stop"
                        fwf = True

                    elif self.leave_fw(theta_gtg, theta_ao, d_t, d_t1):
                        print("Transitioning to NavigateToGoal from WallFollower")
                        self.current_state = "NavigateToGoal"
                        fwf = True

                    else:
                        d_t1 = np.sqrt((self.x_target - self.x)**2 + (self.y_target - self.y)**2) if fwf else d_t1
                        clk_cnt = self.clockwise_counter(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle) if fwf else clk_cnt
                        fwf = False
                        v_wf, w_wf = self.fw_controller(closest_angle, clk_cnt)
                        v_msg.linear.x = v_wf
                        v_msg.angular.z = w_wf

            self.pub_cmd_vel.publish(v_msg)

            rate.sleep()

    def at_goal(self):
        # This function checks if the robot is within the target position tolerance
        return np.sqrt((self.x_target - self.x)**2 + (self.y_target - self.y)**2) < self.target_position_tolerance

    def get_closest_object(self, lidar_msg):
        # This function returns the closest object range and angle from the lidar data
        min_idx = np.argmin(lidar_msg.ranges)
        closest_range = lidar_msg.ranges[min_idx]
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))

        return closest_range, closest_angle

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
        # This function calculates the linear and angular speeds to navigate towards a goal
        kvmax = 0.2  # max linear speed gain
        kwmax = 1.0  # max angular speed gain

        av = 1.0  # Exponential growth rate adjustment for linear speed
        aw = 2.0  # Exponential growth rate adjustment for angular speed

        ed = np.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)

        # Calculate angle to the target position
        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot

        # Normalize e_theta to be within -pi and pi
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        # Calculate the robot's angular speed
        kw = kwmax * (1 - np.exp(-aw * e_theta**2)) / abs(e_theta)
        w = kw * e_theta

        if abs(e_theta) > np.pi / 8:
            # Rotate to face the goal
            v = 0  # No linear speed
        else:
            # Adjust linear speed gain proportional to the distance to the target
            kv = kvmax * (1 - np.exp(-av * ed**2)) / abs(ed)
            v = kv * ed

        return v, w

    def clockwise_counter(self, x_target, y_target, x_robot, y_robot, theta_robot, closest_angle):
        # This function determines the direction (clockwise or counterclockwise) to follow the wall
        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao -= np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi / 2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        if np.abs(theta_fw - e_theta) <= np.pi / 2:
            return 1
        else:
            return 0

    def compute_angles(self, x_target, y_target, x_robot, y_robot, theta_robot, closest_angle):
        # This function calculates the angles for the goal and the obstacle
        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao -= np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        return e_theta, theta_ao

    def fw_controller(self, closest_angle, counter_clockwise):
        # This function calculates the wall-following control commands
        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao -= np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi / 2 + theta_ao if counter_clockwise else np.pi / 2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        w_fw = 2.0 * theta_fw
        v_fw = 0.09

        return v_fw, w_fw

    def leave_fw(self, theta_gtg, theta_ao, d_t, d_t1):
        # This function decides if the robot should leave wall-following mode
        print(f"Distance to target (previous): {d_t1}")
        print(f"Distance to target (current): {d_t}")
        print(f"Angle clearance: {np.abs(theta_ao - theta_gtg)}")
        print()
        if (d_t < d_t1) and (np.abs(theta_ao - theta_gtg) < np.pi / 2):
            return True
        else:
            return False

    def laser_cb(self, msg):
        # Callback function to receive lidar data
        self.lidar_msg = msg
        self.lidar_received = True

    def wl_cb(self, wl):
        # Callback function to receive left wheel speed [rad/s]
        self.wl = wl.data

    def wr_cb(self, wr):
        # Callback function to receive right wheel speed [rad/s]
        self.wr = wr.data

    def goal_cb(self, goal):
        # Callback function to receive goal position from RViz
        print("Received new goal position.")
        self.x_target = goal.pose.position.x
        self.y_target = goal.pose.position.y
        self.goal_received = True

    def update_state(self, wr, wl, delta_t):
        # Update the robot's state based on wheel speeds
        v = self.r * (wr + wl) / 2
        w = self.r * (wr - wl) / self.L

        self.theta += w * delta_t

        # Normalize theta to be within -pi and pi
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        vx = v * np.cos(self.theta)
        vy = v * np.sin(self.theta)

        self.x += vx * delta_t
        self.y += vy * delta_t

    def cleanup(self):
        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)

############################### MAIN PROGRAM ####################################

if __name__ == "__main__":
    rospy.init_node("gtg_obstacles", anonymous=True)
    NavigateToGoal()
    rospy.spin()