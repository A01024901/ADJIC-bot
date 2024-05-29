#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np

class AutonomousNav():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        ############ ROBOT CONSTANTS ################

        self.r = 0.05 # wheel radius [m]
        self.L = 0.19 # wheel separation [m]

        ############ Variables ###############

        self.x = 0.0 # x position of the robot [m]
        self.y = 0.0 # y position of the robot [m]
        self.theta = 0.0 # angle of the robot [rad]

        ############ Variables ###############

        self.path_points = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]  # Defining the waypoints
        self.current_point_index = 0  # Index of the current waypoint
        self.x_target, self.y_target = self.path_points[self.current_point_index]  # Target position
        self.goal_received = True  # Since we're not waiting for goals, set it to True
        self.lidar_received = False  # Flag to indicate if the laser scan has been received
        self.target_position_tolerance = 0.1  # Acceptable distance to the goal to declare the robot has arrived [m]
        fw_distance = 0.3
        self.integral = 0.0
        self.prev_error = 0.0
        stop_distance = 0.10  # Distance from closest obstacle to stop the robot [m]
        v_msg = Twist()  # Robot's desired speed
        self.wr = 0  # Right wheel speed [rad/s]
        self.wl = 0  # Left wheel speed [rad/s]
        self.current_state = 'GoToGoal'  # Robot's current state

        ###******* INIT PUBLISHERS *******###
        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)

        ############################### SUBSCRIBERS #####################################

        rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)
        rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)
        rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("puzzlebot_1/base_controller/odom", Odometry, self.odom_cb)

        #********** INIT NODE **********###
        freq = 20
        rate = rospy.Rate(freq)  # freq Hz
        dt = 1.0 / float(freq)  # Dt is the time between one calculation and the next one
        fwf = True
        d_t1 = 0.0
        d_t = 0.0
        theta_ao = 0.0
        theta_gtg = 0.0

        ################ MAIN LOOP ################
        while not rospy.is_shutdown():
            if not self.lidar_received:
                rospy.loginfo("Waiting for lidar data...")
                rate.sleep()
                continue

            closest_range, closest_angle = self.get_closest_object(self.lidar_msg)  # Get the closest object range and angle

            if self.current_state == 'Stop':
                if self.current_point_index < len(self.path_points):
                    self.x_target, self.y_target = self.path_points[self.current_point_index]
                    rospy.loginfo("Change to GoToGoal from Stop")
                    self.current_state = "GoToGoal"
                else:
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = 0.0
                    rospy.loginfo("Reached final goal. Stopping.")
                    self.pub_cmd_vel.publish(v_msg)

           

            elif self.current_state == 'GoToGoal':
                distance_to_target = self.calculate_distance(self.x_target, self.y_target, self.x, self.y)
                print(f"Distance to current point: {distance_to_target:.2f} meters")
                
                if self.at_goal() or closest_range < stop_distance:
                    if self.current_point_index < len(self.path_points) - 1:
                        self.current_point_index += 1
                        self.x_target, self.y_target = self.path_points[self.current_point_index]
                        print(f"Moving to next point: ({self.x_target}, {self.y_target})")
                    else:
                        print(f"Reached final point: ({self.x_target}, {self.y_target}). Transitioning to Stop.")
                        self.current_state = "Stop"

                elif closest_range < fw_distance:
                    print("Transitioning to WallFollower from GoToGoal")
                    self.current_state = "WallFollower"

                else:
                    v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.x, self.y, self.theta)
                    v_msg.angular.z = w_gtg
                    rospy.sleep(0.2)
                    v_msg.linear.x = v_gtg

            elif self.current_state == 'WallFollower':
                theta_gtg, theta_ao = self.compute_angles(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle)
                d_t = self.calculate_distance(self.x_target, self.y_target, self.x, self.y)
                print(f"Evaluating transition from WallFollower. Distance to target: {d_t:.2f}")

                if self.at_goal() or closest_range < stop_distance:
                    if self.current_point_index < len(self.path_points) - 1:
                        self.current_point_index += 1
                        self.x_target, self.y_target = self.path_points[self.current_point_index]
                        print(f"Moving to next point: ({self.x_target}, {self.y_target})")
                    else:
                        print("Transitioning to Stop")
                        self.current_state = "Stop"
                    fwf = True
                elif self.quit_fw_bug_two(theta_gtg, theta_ao, self.x_target, self.y_target, self.x, self.y):
                    rospy.loginfo("Change to GoToGoal from WallFollower")
                    self.current_state = "GoToGoal"
                    fwf = True
                else:
                    d_t1 = self.calculate_distance(self.x_target, self.y_target, self.x, self.y) if fwf else d_t1
                    clk_cnt = self.clockwise_counter(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle) if fwf else clk_cnt
                    fwf = False
                    v_fw, w_fw = self.following_walls(closest_angle, clk_cnt)
                    v_msg.linear.x = v_fw
                    v_msg.angular.z = w_fw

            self.pub_cmd_vel.publish(v_msg)
            rate.sleep()

    def at_goal(self):
        # This function checks if the robot is within the target position tolerance
        distance = self.calculate_distance(self.x_target, self.y_target, self.x, self.y)
        if distance < self.target_position_tolerance:
            print(f"Reached point: ({self.x_target}, {self.y_target})")
            return True
        else:
            return False

    def calculate_distance(self, x_target, y_target, x_robot, y_robot):
        return np.sqrt((x_target - x_robot)**2 + (y_robot - y_target)**2)

    def following_walls(self, closest_angle, counter_clockwise):
        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi / 2 + theta_ao if counter_clockwise else np.pi / 2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        w_fw = 2.0 * theta_fw
        v_fw = 0.09

        return v_fw, w_fw

    def get_closest_object(self, lidar_msg):
        min_idx = np.argmin(lidar_msg.ranges)
        closest_range = lidar_msg.ranges[min_idx]
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))

        return closest_range, closest_angle

    def is_near_enough(self, x1, y1, x2, y2, x3, y3, epsilon):
        a, b, c = self.line_equation(x2, y2, x3, y3)
        numerator = np.abs(a * x1 + b * y1 + c)
        denominator = np.sqrt(a * a + b * b)
        distance = numerator / denominator

        if distance <= epsilon:
            return True
        return False

    def line_equation(self, x1, y1, x2, y2):
        a = y2 - y1
        b = x1 - x2
        c = x2 * y1 - x1 * y2

        return a, b, c

    def quit_fw_bug_two(self, theta_gtg, theta_ao, x_target, y_target, x_robot, y_robot):
        n_segment = self.is_near_enough(x_robot, y_robot, 0, 0, x_target, y_target, 0.08)
        if (n_segment) and (np.abs(theta_ao - theta_gtg) < np.pi / 2):
            return True
        else:
            return False

    def quit_fw_bug_zero(self, theta_gtg, theta_ao, d_t, d_t1):
        if (d_t < d_t1) and (np.abs(theta_ao - theta_gtg) < np.pi / 2):
            return True
        else:
            return False

    def clockwise_counter(self, x_target, y_target, x_robot, y_robot, theta_robot, closest_angle):
        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi / 2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        if np.abs(theta_fw - e_theta) <= np.pi / 2:
            return 1
        else:
            return 0

    def compute_angles(self, x_target, y_target, x_robot, y_robot, theta_robot, closest_angle):
        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        return e_theta, theta_ao

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
        # This function returns the linear and angular speed to reach a given goal
        # This functions receives the goal's position (x_target, y_target) [m]
        # and robot's position (x_robot, y_robot, theta_robot) [m, rad]
        # This functions returns the robot's speed (v, w) [m/s] and [rad/s]

        kvmax = 0.7  # Linear speed maximum gain
        kwmax = 1.8  # Angular speed maximum gain

        av = 1.0  # Constant to adjust the exponential's growth rate
        aw = 2.0  # Constant to adjust the exponential's growth rate

        ed = np.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)

        # Compute angle to the target position

        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot

        # Limit e_theta from -pi to pi
        # This part is very important to avoid abrupt changes when error switches between 0 and +-2pi
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
        # Compute the robot's angular speed
        kw = kwmax * (1 - np.exp(-aw * e_theta**2)) / abs(e_theta)  # Constant to change the speed
        w = kw * e_theta

        if abs(e_theta) > np.pi / 8:
            # We first turn to the goal
            v = 0  # Linear speed

        else:
            # Make the linear speed gain proportional to the distance to the target position
            kv = kvmax * (1 - np.exp(-av * ed**2)) / abs(ed)  # Constant to change the speed
            v = kv * ed  # Linear speed

        return v, w

    def laser_cb(self, msg):
        ## This function receives a message of type LaserScan
        self.lidar_msg = msg
        self.lidar_received = True

    def wl_cb(self, wl):
        ## This function receives the left wheel speed [rad/s]
        self.wl = wl.data

    def wr_cb(self, wr):
        ## This function receives the right wheel speed [rad/s]
        self.wr = wr.data

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion(orientation_list)

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

    def cleanup(self):
        # This function is called just before finishing the node
        # You can use it to clean things up before leaving
        # Example: stop the robot before finishing a node.

        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)

############################### MAIN PROGRAM ####################################

if __name__ == "__main__":
    rospy.init_node("gtg_obstacles", anonymous=True)
    AutonomousNav()
    rospy.spin()