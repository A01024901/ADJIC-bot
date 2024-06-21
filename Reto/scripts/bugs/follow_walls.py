#!/usr/bin/env python3 

import rospy 
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from utils import PIDController
from geometry_msgs.msg import PointStamped , Twist , PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class follow_walls:
    def __init__(self , mode , bug):
        ###--- Modo del Nodo ---###
        if mode == "sim":
            scan = "/puzzlebot_1/scan"

        elif mode == "real":
            scan = "/scan"

        if bug == "cero":
            gains_a = [1.9, 0.002, 0.001]
            gains_d = [2.0, 0.001, 0.001]

        elif bug == "two":
            gains_a = [2.3, 0.0015, 0.001]
            gains_d = [2.05, 0.001, 0.001]

        ###--- Inicio del Nodo ---###
        rospy.init_node('follow_wall')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber(scan, LaserScan, self.laser_cb)
        rospy.Subscriber("/odom" , Odometry , self.odom_cb) 
        rospy.Subscriber("/target" , PointStamped , self.target_cb)

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('/bug_twist', Twist, queue_size=1)
        self.pub_bug_vector = rospy.Publisher('/bug_vector', PoseStamped, queue_size=1) 
        self.pub_flag_front = rospy.Publisher('/front_object', Bool, queue_size=1) 
        self.pub_flag_clear = rospy.Publisher('/clear_path', Bool, queue_size=1) 

        ###--- Constants ---###
        self.follow_distance = 0.17
        self.safe_distance = 0.07
        self.dt = 0.02
        self.k_w_AO = 0.4
        self.k_w_C_AO = 2.8
        self.v_C_AO = 0.15

        ###--- Objetos ---###
        self.controller_a = PIDController(gains_a)
        self.controller_b = PIDController(gains_d)
        self.cmd_vel = Twist() 
        self.bug_vector = PoseStamped()
        self.scan = LaserScan()
        self.object_flag = Bool()
        self.clear_flag = Bool()
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.scan_received = False
        self.crash_state = False
        self.distance_tg = 0
        self.cl_angle = 0
        self.cl_distance = 0

        self.x_pos = 0
        self.y_pos = 0
        self.theta_robot = 0

        self.x_target = 0
        self.y_target = 0

        self.v = 0
        self.w = 0
        self.fw_th = 0

        while rospy.get_time() == 0: pass 

        while not rospy.is_shutdown():
            if self.scan_received:
                self.eval_conditions()

                if self.safe_zone():
                    print("Safe_Zone")
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                
                else: self.calc_fw()

            self.print_state()
            self.fill_msgs()
                
            ###--- Publish ---###
            self.pub_cmd_vel.publish(self.cmd_vel)
            self.pub_flag_front.publish(self.object_flag)
            self.pub_bug_vector.publish(self.bug_vector)
            self.pub_flag_clear.publish(self.clear_flag)
            rate.sleep()
    
    #######------ Calculations ------######

    def eval_rotation(self):
        self.th_tg = np.arctan2((self.y_target - self.x_pos), (self.x_target - self.y_pos))
        self.distance_tg = np.sqrt(((self.x_target - self.x_pos) ** 2) + ((self.y_target - self.y_pos) ** 2))
        fw_th = (np.pi / 2) + self.cl_angle
        fw_th = np.arctan2(np.sin(fw_th), np.cos(fw_th))  
        if (self.cl_distance < self.follow_distance) and (abs(fw_th - self.th_tg) <= (np.pi / 2.0) - 0.2) :
            return True # = Counter Clock Wise
        elif (self.cl_distance < self.follow_distance) and (abs(fw_th - self.th_tg) > (np.pi / 2.0) - 0.2) :
            return False # = Clock Wise
        
    def calc_fw(self , target_distance):
        cw_b = self.eval_rotation()
        self.fw_th = (np.pi / 2) + self.cl_angle if cw_b else -(np.pi / 2) + self.cl_angle

        theta_ao = self.cl_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        fw_th = -np.pi/2 + theta_ao if self.fw_th else np.pi/2 + theta_ao
        fw_th = np.arctan2(np.sin(fw_th), np.cos(fw_th))
        a_control = self.controller_a.cacl(fw_th)

        d_error = self.cl_distance - target_distance
        d_control = self.controller_b.cacl(d_error)

        w_fw = a_control + d_control  
        v_fw = 0.08

        self.v = v_fw
        self.w = w_fw

    ######------ Conditions -----#########

    def eval_conditions(self):
        self.distance_tg = np.sqrt(((self.x_target - self.x_pos) ** 2) + ((self.y_target - self.y_pos) ** 2))
        distance = self.cl_distance <= self.follow_distance
        angle = self.cl_angle > -np.pi/3 and self.cl_angle < np.pi/3
        return distance and angle
    
    def enough_turn(fw_t , current_t):
        return (current_t - fw_t) > 1

    def clear_path(self):
        angle_to_goal = np.arctan2((self.y_target - self.y_pos), (self.x_target - self.x_pos))
        angle_to_goal = np.arctan2(np.sin(angle_to_goal), np.cos(angle_to_goal))
        index_goal_angle = int((angle_to_goal - self.scan.angle_min) / self.scan.angle_increment)
        if 0 <= index_goal_angle < len(self.scan.ranges):
            distance_to_goal = np.sqrt((self.x_target - self.x_pos)**2 + (self.y_target - self.y_pos)**2)
            if self.scan.ranges[index_goal_angle] > distance_to_goal:
                self.clear_flag.data = True

            else:
                self.clear_flag.data = False

    def safe_zone(self):
        distance = self.cl_distance <= self.safe_distance
        angle = -np.pi/4 < self.cl_angle < np.pi/4
        return distance and angle
    
    ####----- Prints and messages -----#####

    def fill_msgs(self):
        self.cmd_vel.linear.x = self.v
        self.cmd_vel.angular.z = self.w

        quat = quaternion_from_euler(0, 0, self.fw_th)
        self.bug_vector.header.frame_id = "odom"
        self.bug_vector.pose.position.x = self.x_pos
        self.bug_vector.pose.position.y = self.y_pos
        self.bug_vector.pose.orientation.x = quat[0]
        self.bug_vector.pose.orientation.y = quat[1]
        self.bug_vector.pose.orientation.z = quat[2]
        self.bug_vector.pose.orientation.w = quat[3]

        self.object_flag.data = self.crash_state
        self.clear_flag.data = False

    def print_state(self):
        print("###################################")
        print("Posicion robot: " , self.x_pos , self.y_pos)
        print("Velocidades: " , self.v , self.w)
        print("Angulo FW: " , self.fw_th)
        print("Angulo CL: " , self.cl_angle)
        print("Crash State: " , self.crash_state)
        print("Clear flag: " , self.clear_flag)
        print("###################################")

    ####---- Callbacks -----#####

    def odom_cb(self , msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.theta_robot = euler_from_quaternion([x, y, z, w])
    
    def target_cb(self , msg):
        self.x_target = msg.point.x
        self.y_target = msg.point.y

    def laser_cb (self , msg):
        self.scan_received = True
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        self.cl_distance = valid_ranges.min() if valid_ranges.size > 0 else float('inf')
        idx = ranges.tolist().index(self.cl_distance) if valid_ranges.size > 0 else 0
        cl_angle = msg.angle_min + idx * msg.angle_increment
        self.cl_angle = np.arctan2(np.sin(cl_angle), np.cos(cl_angle))

    def cleanup (self):
        print ("Apagando Localsation")
        self.pub_cmd_vel.publish(Twist())
        self.pub_flag_front.publish(Bool())        
        
if __name__ == "__main__": 
    follow_walls("sim" , "two")