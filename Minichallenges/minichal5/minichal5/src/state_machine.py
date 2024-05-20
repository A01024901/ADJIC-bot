#!/usr/bin/env python3 
import rospy 
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

class state_machine:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('state_machine')
        rospy.on_shutdown(self.cleanup)

        ###--- Subscriptores  GTG---###
        rospy.Subscriber("gtg_twist" , Twist , self.gtg_cb)
        rospy.Subscriber("pos_gtg" , PoseStamped , self.pos_gtg_cb)
        rospy.Subscriber("target_gtg" , PoseStamped , self.target_gtg_cb)
        rospy.Subscriber("at_goal_flag" , Bool , self.flag_gtg_cb)

        ###-- Subscriptores FW ---###
        rospy.Subscriber("fw_twist" , Twist , self.fw_cb)
        rospy.Subscriber("front_object" , Bool , self.fw_f_cb)

        ###--- Publishers ---###
        self.cmd_vel_pub = rospy.Publisher("puzzlebot_1/base_controller/cmd_vel" , Twist , queue_size=1)

        ###--- Compare ---###
        self.x_pos = 0
        self.y_pos = 0
        self.x_goal = 0
        self.y_goal = 0
        self.x_temp = 0
        self.y_temp = 0

        ###--- Variables ---###
        self.gtg_flag = False
        self.object_front = False
        self.pos = 0
        self.state = "go_to_goal" #"avoid_obstacle" "Stop"

        ###--- Constants ---###
        self.dt = 0.02

        ###--- Objects ---###
        self.cmd_vel = Twist()
        self.gtg_vel = Twist()
        self.fw_vel = Twist()
        rate = rospy.Rate(int(1.0/self.dt))

        while rospy.get_time() == 0: print ("Simulacion no iniciada") #Descomentar en simulacion 

        print("Nodo operando")

        while not rospy.is_shutdown():
            if self.state == "on_hold":
                print ("On_hold")   
                self.hold_behave()

            if self.state == "go_to_goal":
                print ("Go to goal" , "X:" , self.x_pos , "Y:" , self.y_pos)
                print ("Go to goal" , "XP:" , self.x_goal , "YP:" , self.y_goal)
                self.gtg_behave()

            elif self.state == "follow_wall":
                print ("Follow Wall")
                self.follow_wall_behave()

            elif self.state == "stop":
                print ("Reached goal")
                self.stop_behave()

            ###--- Publish ---###
            self.cmd_vel_pub.publish(self.cmd_vel)
            rate.sleep()

    def hold_behave(self):
        self.cmd_vel = Twist()
        if self.x_pos != self.x_goal:
            self.state = "go_to_goal"

    def gtg_behave(self):
        self.cmd_vel = self.gtg_vel
        if self.gtg_flag:
            self.state = "stop"
        elif (not self.gtg_flag) and self.object_front:
            self.state = "follow_wall"
            self.x_temp = self.x_pos
            self.y_temp = self.y_pos

    def follow_wall_behave(self):
        self.cmd_vel = self.fw_vel
        if self.gtg_flag:
            self.state = "stop"

        elif self.calc_distance():
            self.state = "go_to_goal"

    def calc_distance(self):
        distance_temp = np.sqrt((self.x_goal - self.x_temp)**2 + (self.y_goal - self.y_temp)**2)
        distance_real = np.sqrt((self.x_goal - self.x_pos)**2 + (self.y_goal - self.y_pos)**2)
        if distance_real < distance_temp: 
            return True
        else: 
            return False

    def stop_behave(self):
        self.cmd_vel = Twist()

    def gtg_cb(self , twist):
        self.gtg_vel = twist

    def pos_gtg_cb(self , msg):
        self.x_pos = msg.pose.position.x 
        self.y_pos = msg.pose.position.y

    def target_gtg_cb(self , msg):
        self.x_goal = msg.pose.position.x 
        self.y_goal = msg.pose.position.y

    def flag_gtg_cb(self , msg):
        self.gtg_flag = msg.data

    def fw_cb(self , twist):
        self.fw_vel = twist   
    
    def fw_f_cb(self , msg):
        self.object_front = msg.data   

    def cleanup (self):
        print ("Apagando Localsation")

if __name__ == "__main__": 
    state_machine()