#!/usr/bin/env python3 

import rospy 
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
class puzzlebot_sim:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('puzzlebot_sim')
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(30)
        #self.camera = cv2.VideoCapture(0)

        ###--- Parametros ---###
        

        ###--- Objetos ---###
        self.cmd_vel_flag = False

        ###--- Subscriptores ---###
        rospy.Subscriber("/cmd_vel" , Twist , self.vel_cb)

        ###--- Publishers ---###
        self.pose_pub = rospy.Publisher("pose" , Point , queue_size=5)
        self.wr_pub = rospy.Publisher("wr" , Float32 , queue_size=10)
        self.wl_pub = rospy.Publisher("wl" , Float32 , queue_size=10)

        while rospy.get_time() == 0: print ("Simulacion no iniciada")

        print("Camara operando")

        while not rospy.is_shutdown():
            
            flag , tvecs , rvecs = self.aruco_finder()

            if flag:
                self.pos_aruco.x = tvecs[0]
                self.pos_aruco.y = tvecs[1]
                self.pos_aruco.z = tvecs[2]
                self.aruco_flag_pub.publish(flag)
                self.pos_aruco_pub.publish(self.pos_aruco)
            
            else: 
                self.aruco_flag_pub.publish(flag)

    def simulate_pos():
        pass
        
    def vel_cb (self , data):
        try:
            self.img = self.bd_object.imgmsg_to_cv2(data , desired_encoding="bgr8")
            self.img_flag = True
        except:
            print ("No hay ")

    def cleanup (self):
        print ("Apagando Camara")
        

if __name__ == "__main__": 
    camara()