#!/usr/bin/env python3 

import rospy 
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class follow_walls:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('follow_wall')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores ---###
        rospy.Subscriber("/puzzlebot_1/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("pos_gtg" , PoseStamped , self.pos_gtg_cb) 
        rospy.Subscriber("target_gtg" , PoseStamped , self.target_gtg_cb)

        ###--- Publishers ---###
        self.pub_cmd_vel = rospy.Publisher('fw_twist', Twist, queue_size=1) 
        self.pub_flag_front = rospy.Publisher('front_object', Bool, queue_size=1) 
        self.pub_flag_clear = rospy.Publisher('front_clear', Bool, queue_size=1) 

        ###--- Constants ---###
        self.follow_distance = 0.25
        self.safe_distance = 0.07
        self.dt = 0.02
        self.k_w_AO = 0.4
        self.k_w_C_AO = 2.8
        self.v_C_AO = 0.15

        ###--- Objetos ---###
        self.cmd_vel = Twist() 
        self.scan = LaserScan()
        self.object_flag = Bool()
        self.clear_flag = Bool()
        rate = rospy.Rate(int(1.0/self.dt))

        ###--- Variables ---###
        self.scan_flag = False
        self.crash_state = False
        self.distance_temp = 0
        self.x_pos = 0
        self.y_pos = 0
        self.x_goal = 0
        self.y_goal = 0
        self.th_goal = 0

        while rospy.get_time() == 0: pass 

        while not rospy.is_shutdown():
            if self.scan.ranges:
                d = np.sqrt(((self.x_goal - self.x_pos) ** 2) + ((self.y_goal - self.y_pos) ** 2))
                self.scan_flag = False
                closest_distance = min(self.scan.ranges)
                index = self.scan.ranges.index(closest_distance)
                closest_angle = self.scan.angle_min + index * self.scan.angle_increment
                closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))  
                self.eval_conditions(closest_distance , closest_angle)
                
                self.object_flag.data = self.crash_state

                self.th_goal = np.arctan2((self.y_goal - self.x_pos), (self.x_goal - self.y_pos))

                rot = self.eval_rotation(closest_distance , closest_angle)

                state = self.safe_zone(closest_distance , closest_angle)
                print (closest_distance)

                if self.safe_zone(closest_distance , closest_angle):
                    print("Safe_Zone")
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.25 * rot
                
                else: self.calc_fw(rot , closest_angle)
                
            ###--- Publish ---###
            self.pub_cmd_vel.publish(self.cmd_vel)
            self.pub_flag_front.publish(self.object_flag)
            self.pub_flag_clear.publish(self.clear_flag)
            rate.sleep()

    def eval_conditions(self , distance , angle):
        distance = distance <= self.follow_distance
        angle = angle > -np.pi/4 and angle < np.pi/4
        if distance and angle: 
            self.crash_state = True

    def safe_zone(self , distance , angle):
        distance = distance <= self.safe_distance
        angle = angle > -np.pi/4 and angle < np.pi/4
        return distance

    def eval_rotation(self , cl_distance , cl_angle):
        fw_th = (np.pi / 2) + cl_angle
        fw_th = np.arctan2(np.sin(fw_th), np.cos(fw_th))  
        if (cl_distance < self.follow_distance) and (abs(fw_th - self.th_goal) <= (np.pi / 2.0) - 0.2) :
            return True # = Counter Clock Wise
        elif (cl_distance < self.follow_distance) and (abs(fw_th - self.th_goal) > (np.pi / 2.0) - 0.2) :
            return False # = Clock Wise
        
    def calc_fw(self , cw_b , closest_angle):
        #print ("Follow W")
        fw_th = (np.pi / 2) + closest_angle if cw_b else -(np.pi / 2) + closest_angle
        if (-0.08 > fw_th > 0.08) and self.crash_state :
            v_AO = 0.0
            w_AO = self.k_w_AO * fw_th
        else :
            v_AO = self.v_C_AO
            w_AO = self.k_w_C_AO * fw_th

        self.cmd_vel.linear.x = v_AO
        self.cmd_vel.angular.z = w_AO
        print ("Calc_vel: " ,v_AO , w_AO)

    def clear_path(self):
        angle_to_goal = np.arctan2((self.y_goal - self.y_pos), (self.x_goal - self.x_pos))
        
        # Normalizar el ángulo
        angle_to_goal = np.arctan2(np.sin(angle_to_goal), np.cos(angle_to_goal))
        
        # Calcular el índice del LIDAR correspondiente al ángulo hacia el objetivo
        index_goal_angle = int((angle_to_goal - self.scan.angle_min) / self.scan.angle_increment)
        
        # Verificar si el índice está dentro del rango del LIDAR
        if 0 <= index_goal_angle < len(self.scan.ranges):
            # Verificar si la distancia medida en esa dirección es mayor que la distancia al objetivo
            distance_to_goal = np.sqrt((self.x_goal - self.x_pos)**2 + (self.y_goal - self.y_pos)**2)
            if self.scan.ranges[index_goal_angle] > distance_to_goal:
                self.clear_flag.data = True
        self.clear_flag.data = False

    def wl_cb (self , msg):
        self.wl = msg.data

    def pos_gtg_cb(self , msg):
        self.x_pos = msg.pose.position.x 
        self.y_pos = msg.pose.position.y
    
    def target_gtg_cb(self , msg):
        self.x_goal = msg.pose.position.x 
        self.y_goal = msg.pose.position.y
    def laser_cb (self , msg):
        self.scan = msg
        data = msg.ranges
        #self.scan.ranges = np.roll(data, int(len(data)/2 + 1))   

    def cleanup (self):
        print ("Apagando Localsation")
        self.pub_cmd_vel.publish(Twist())
        self.pub_flag_front.publish(Bool())        
        
if __name__ == "__main__": 
    follow_walls()