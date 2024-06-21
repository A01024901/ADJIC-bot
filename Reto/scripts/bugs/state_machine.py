#!/usr/bin/env python
import rospy 
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class state_machine:
    def __init__(self , mode , bug):
        ###--- Modo del Nodo ---###
        if mode == "sim":
            cmd_vel_pub = "puzzlebot_1/base_controller/cmd_vel"

        elif mode == "real":
            cmd_vel_pub = "/cmd_vel"

        if bug == "zero":
            self.quit_fw = "puzzlebot_1/base_controller/cmd_vel"

        elif bug == "twp":
            self.quit_fw = "/cmd_vel"        

        ###--- Inicio del Nodo ---###
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores Odom---### 
        rospy.Subscriber("/odom" , Odometry , self.odom_cb)

        ###--- Subscriptores  GTG---###
        rospy.Subscriber("/gtg_twist" , Twist , self.gtg_cb)
        rospy.Subscriber("/finish_path" , Bool , self.finish_path_cb)

        ###-- Subscriptores FW ---###
        rospy.Subscriber("/bug_twist" , Twist , self.fw_cb)
        rospy.Subscriber("/front_object" , Bool , self.fw_f_cb)
        rospy.Subscriber("/front_clear" , Bool , self.fw_clear_cb)

        ###--- Publishers ---###
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_pub , Twist , queue_size=1)
        self.clear_flag = rospy.Publisher("clear_obstacle" , Bool , queue_size=1)

        ###--- Compare ---###
        self.x_pos = 0
        self.y_pos = 0
        self.x_goal = 0
        self.y_goal = 0
        self.x_temp = 0
        self.y_temp = 0

        ###--- Variables ---###
        self.finish_path = False
        self.object_front = False
        self.clear_path = False
        self.pos = 0
        self.state = "go_to_goal" #"avoid_obstacle" "Stop"

        ###--- Constants ---###
        self.dt = 0.02

        ###--- Objects ---###
        self.cmd_vel = Twist()
        self.gtg_vel = Twist()
        self.fw_vel = Twist()
        rate = rospy.Rate(int(1.0/self.dt))

        while rospy.get_time() == 0: pass #Descomentar en simulacion 

        print("Maquina de estados")

        while not rospy.is_shutdown():

            if self.state == "go_to_goal":
                self.gtg_behave()

            elif self.state == "follow_wall":
                self.follow_wall_behave()

            elif self.state == "stop":
                self.stop_behave()

            self.print_states()

            ###--- Publish ---###
            self.cmd_vel_pub.publish(self.cmd_vel)
            rate.sleep()

    def gtg_behave(self):
        self.cmd_vel = self.gtg_vel
        if self.finish_path:
            self.state = "stop"
        elif (not self.finish_path) and self.object_front:
            self.state = "follow_wall"
            self.x_temp = self.x_pos
            self.y_temp = self.y_pos

    def follow_wall_behave(self):
        self.cmd_vel = self.fw_vel
        if self.finish_path:
            self.state = "stop"

        elif self.clear_path: #Clearshot
            self.state = "go_to_goal"

        elif self.calc_distance() and not self.object_front: #Bug0
            self.state = "go_to_goal"

    def stop_behave(self):
        self.cmd_vel = Twist()

    def calc_distance(self):
        distance_temp = np.sqrt((self.x_goal - self.x_temp)**2 + (self.y_goal - self.y_temp)**2)
        distance_real = np.sqrt((self.x_goal - self.x_pos)**2 + (self.y_goal - self.y_pos)**2)
        distance = distance_real < distance_temp
        
        if distance: 
            return True
        else: 
            return False
        
    def quit_fw_zero(self):
        return self.calc_distance() and not self.object_front
    
    def quit_fw_two(self):
        return self.calc_distance() and not self.object_front

    def print_states(self):
        print("###############################")
        print("State: " , self.state)
        print("Position x: " , round(self.x_pos , 3) , "Target x: " , self.x_goal)
        print("Position y: " , round(self.y_pos , 3) , "Target y: " , self.y_goal)
        print("Temp x: " , round(self.x_temp , 3) , "Temp y: " , round(self.y_temp , 3))
        print("GTGF: " , self.finish_path , " Object: " , self.object_front , " Clear: " , self.clear_path)
        print("GTGF: " , self.finish_path , " Object: " , self.object_front , " Clear: " , self.clear_path)
        print("###############################")

    def gtg_cb(self , twist):
        self.gtg_vel = twist

    def odom_cb(self , msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.theta_robot = euler_from_quaternion([x, y, z, w])

    def target_gtg_cb(self , msg):
        self.x_goal = msg.pose.position.x 
        self.y_goal = msg.pose.position.y

    def finish_path_cb(self , msg):
        self.finish_path = msg.data

    def fw_cb(self , twist):
        self.fw_vel = twist   

    def fw_clear_cb(self , msg):
        self.clear_path = msg.data  

    def fw_f_cb(self , msg):
        self.object_front = msg.data   

    def cleanup (self):
        print ("Apagando Localsation")

if __name__ == "__main__": 
    rospy.init_node('state_machine')
    state_machine("real")