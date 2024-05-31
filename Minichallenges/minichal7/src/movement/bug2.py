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

        self.r = 0.05
        self.L = 0.19

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.path_points = [(1.0, 1.0), (2.0, 1.0), (0.0, 0.0)]
        self.current_point_index = 0
        self.x_target, self.y_target = self.path_points[self.current_point_index]
        self.goal_received = True
        self.lidar_received = False
        self.target_position_tolerance = 0.1
        self.fw_distance = 0.5
        self.stop_distance = 0.05
        self.epsilon = 0.1
        self.obstacle_detected = False
        self.v_msg = Twist()
        self.wr = 0
        self.wl = 0
        self.current_state = 'GoToGoal'
        self.transition_time = rospy.Time.now()
        self.transition_delay = rospy.Duration(0.5)  # 0.5 seconds delay

        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)
        rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)
        rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)

        freq = 20
        rate = rospy.Rate(freq)
        dt = 1.0 / float(freq)
        fwf = True 
        d_t1 = 0.0
        d_t = 0.0
        theta_ao = 0.0
        theta_gtg = 0.0

        while not rospy.is_shutdown():
            if not self.lidar_received:
                rospy.loginfo("Waiting for lidar data...")
                rate.sleep()
                continue

            closest_range, closest_angle = self.get_closest_object(self.lidar_msg)

            if self.current_state == 'Stop':
                if self.current_point_index < len(self.path_points):
                    self.x_target, self.y_target = self.path_points[self.current_point_index]
                    rospy.loginfo("Change to GoToGoal from Stop")
                    self.current_state = "GoToGoal"
                else:
                    self.v_msg.linear.x = 0.0
                    self.v_msg.angular.z = 0.0
                    rospy.loginfo("Reached final goal. Stopping.")
                    self.pub_cmd_vel.publish(self.v_msg)
                    break

            elif self.current_state == 'GoToGoal':
                distance_to_target = self.calculate_distance(self.x_target, self.y_target, self.x, self.y)
                print(f"Distance to current point: {distance_to_target:.2f} meters")
                print(f"theta AO: {theta_ao:.3f}")
                print(f"theta GTG: {theta_gtg:.3f}")
                
                if self.at_goal() or closest_range < self.stop_distance:
                    if self.current_point_index < len(self.path_points) - 1:
                        self.current_point_index += 1
                        self.x_target, self.y_target = self.path_points[self.current_point_index]
                        print(f"Moving to next point: ({self.x_target}, {self.y_target})")
                    else:
                        print(f"Reached final point: ({self.x_target}, {self.y_target}). Transitioning to Stop.")
                        self.current_state = "Stop"

                elif closest_range < self.fw_distance and rospy.Time.now() - self.transition_time > self.transition_delay:
                    print("Transitioning to WallFollower from GoToGoal")
                    self.obstacle_detected = True
                    self.current_state = "WallFollower"
                    self.transition_time = rospy.Time.now()

                else:
                    v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.x, self.y, self.theta)
                    self.v_msg.angular.z = w_gtg
                    self.v_msg.linear.x = v_gtg if abs(self.theta - w_gtg) < np.pi / 4 else 0.0  # Prioritize turning if angle error is large

            elif self.current_state == 'WallFollower':
                theta_gtg, theta_ao = self.compute_angles(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle)
                d_t = self.calculate_distance(self.x_target, self.y_target, self.x, self.y)
                print(f"Evaluating transition from WallFollower. Distance to target: {d_t:.2f}")
                print(f"theta AO: {theta_ao:.3f}")
                print(f"theta GTG: {theta_gtg:.3f}")

                if self.at_goal() or closest_range < self.stop_distance:
                    if self.current_point_index < len(self.path_points) - 1:
                        self.current_point_index += 1
                        self.x_target, self.y_target = self.path_points[self.current_point_index]
                        print(f"Moving to next point: ({self.x_target}, {self.y_target})")
                        self.current_state = "GoToGoal"
                    else:
                        print("Transitioning to Stop")
                        self.current_state = "Stop"
                    fwf = True
                elif self.quit_fw_bug_two(theta_gtg, theta_ao, self.x_target, self.y_target, self.x, self.y) and rospy.Time.now() - self.transition_time > self.transition_delay:
                    rospy.loginfo("Change to GoToGoal from WallFollower")
                    self.current_state = "GoToGoal"
                    self.transition_time = rospy.Time.now()
                    fwf = True
                else:
                    if fwf:
                        d_t1 = self.calculate_distance(self.x_target, self.y_target, self.x, self.y)
                        clk_cnt = self.clockwise_counter(self.x_target, self.y_target, self.x, self.y, self.theta, closest_angle)
                        fwf = False
                    v_fw, w_fw = self.following_walls(closest_angle, clk_cnt)
                    self.v_msg.linear.x = v_fw
                    self.v_msg.angular.z = w_fw

            self.pub_cmd_vel.publish(self.v_msg)
            rate.sleep()

    def at_goal(self):
        distance = self.calculate_distance(self.x_target, self.y_target, self.x, self.y)
        if distance < self.target_position_tolerance:
            print(f"Reached point: ({self.x_target}, {self.y_target})")
            return True
        else:
            return False

    def calculate_distance(self, x_target, y_target, x_robot, y_robot):
        return np.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)

    def following_walls(self, closest_angle, counter_clockwise):
        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))
        theta_ao -= np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi / 2 + theta_ao if counter_clockwise else np.pi / 2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        w_fw = 2.0 * theta_fw
        v_fw = 0.1

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
        n_segment = self.is_near_enough(x_robot, y_robot, 0, 0, x_target, y_target, self.epsilon)
        return n_segment and (np.abs(theta_ao - theta_gtg) < np.pi / 2)

    def clockwise_counter(self, x_target, y_target, x_robot, y_robot, theta_robot, closest_angle):
        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))
        theta_ao -= np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi / 2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        return np.abs(theta_fw - e_theta) <= np.pi / 2

    def compute_angles(self, x_target, y_target, x_robot, y_robot, theta_robot, closest_angle):
        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))
        theta_ao -= np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        return e_theta, theta_ao

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
        kvmax = 0.1
        kwmax = 1.2
        av = 1.0
        aw = 2.0

        ed = np.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)
        theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
        e_theta = theta_target - theta_robot
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        kw = kwmax * (1 - np.exp(-aw * e_theta**2)) / abs(e_theta)
        w = kw * e_theta

        if abs(e_theta) > np.pi / 2:
            clk_cnt = self.clockwise_counter(x_target, y_target, x_robot, y_robot, theta_robot, e_theta)
            rospy.loginfo(f"Recalculated clockwise_counter: {clk_cnt}")
        
        if abs(e_theta) > np.pi / 8:
            v = 0  # Stop linear movement if angle error is large
        else:
            kv = kvmax * (1 - np.exp(-av * ed**2)) / abs(ed)
            v = kv * ed
            v = min(v, 0.2)

        return v, w

    def laser_cb(self, msg):
        self.lidar_msg = msg
        self.lidar_received = True

    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.theta) = euler_from_quaternion(orientation_list)
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

    def cleanup(self):
        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)

if __name__ == "__main__":
    rospy.init_node("gtg_obstacles", anonymous=True)
    AutonomousNav()
    rospy.spin()
