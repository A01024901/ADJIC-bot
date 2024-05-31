#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan  # Lidar
import numpy as np
from tf.transformations import euler_from_quaternion

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

        self.path_points = [(1.0, 1.0), (2.0, 2.0), (3.0, 1.0)]  # List of points to follow
        self.current_point_index = 0  # Index of the current target point
        self.x_target, self.y_target = self.path_points[self.current_point_index]  # Initialize with the first target point
        self.goal_received = False  # Flag goal received
        self.lidar_received = False  # Flag laser scan received
        self.target_position_tolerance = 0.1  # Tolerance goal reached
        self.integral = 0.0
        self.prev_error = 0.0
        fw_distance = 0.30  # Distance to switch to wall-following mode
        stop_distance = 0.10  # Distance to stop the robot if an obstacle is close 
        eps = 0.20  # Safety buffer
        v_msg = Twist() 
        self.wr = 0  # Right wheel speed [rad/s]
        self.wl = 0  # Left wheel speed [rad/s]
        self.current_state = 'NavigateToGoal'  

        ### INITIALIZE PUBLISHERS ###

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        ####################### SUBSCRIBERS #######################

        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("puzzlebot_goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/scan", LaserScan, self.laser_cb)

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
                        print("Transitioning to WallFollower from NavigateToGoal")
                        self.current_state = "WallFollower"

                    else:
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.x, self.y, self.theta)
                        v_msg.linear.x = v_gtg
                        v_msg.angular.z = w_gtg

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

                    elif self.leave_fw(theta_gtg, theta_ao, d_t, d_t1):
                        print("Transitioning to NavigateToGoal from WallFollower")
                        self.current_state = "NavigateToGoal"
                        fwf = True

                    else:
                        d_t1 = self.calculate_distance(self.x_target, self.y_target, self.x, self.y) if fwf else d_t1
                        clk_cnt = self.clockwise_counter(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle) if fwf else clk_cnt
                        fwf = False
                        v_wf, w_wf = self.fw_controller(closest_angle, clk_cnt)
                        v_msg.linear.x = v_wf
                        v_msg.angular.z = w_wf

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
        return np.sqrt((x_target - x_robot)*2 + (y_robot - y_target)*2)

    def get_closest_object(self, lidar_msg):
        # This function returns the closest object range and angle from the lidar data
        min_idx = np.argmin(lidar_msg.ranges)
        closest_range = lidar_msg.ranges[min_idx]
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))

        return closest_range, closest_angle

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
        # This function calculates the linear and angular speeds to navigate towards a goal
        kvmax = 0.6  # max linear speed gain
        kwmax = 1.6  # max angular speed gain

        av = 1.0  # Exponential growth rate adjustment for linear speed
        aw = 2.0  # Exponential growth rate adjustment for angular speed

        ed = self.calculate_distance(x_target, y_target, x_robot, y_robot)

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
        # This function decides the direction to turn (clockwise or counter-clockwise) while following walls
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

    def goal_cb(self, goal):
        # Callback function to receive goal position from RViz
        print("Received new goal position.")
        self.path_points = [(goal.pose.position.x, goal.pose.position.y)]
        self.current_point_index = 0
        self.x_target, self.y_target = self.path_points[self.current_point_index]
        self.goal_received = True

    def odom_cb(self , msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        or_q = msg.pose.pose.orientation
        or_list = [or_q.x , or_q.y , or_q.z , or_q.w]
        _ , _ , theta = euler_from_quaternion(or_list)
        self.theta = np.arctan2(np.sin(theta) , np.cos(theta))

    def cleanup(self):
        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)

############################### MAIN PROGRAM ####################################

if __name__ == "__main__":
    rospy.init_node("gtg_obstacles", anonymous=True)
    NavigateToGoal()
    rospy.spin()