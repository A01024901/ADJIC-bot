#!/usr/bin/env python3 

import rospy 
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge , CvBridgeError

class camara:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('Aruco Detector')
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(30)
        #self.camera = cv2.VideoCapture(0)

        ###--- Parametros Camara ---###
        self.test_camera= np.array([[1000 , 0 , 320] , [0 , 1000 , 240] , [0 , 0 , 1]])
        self.test_coefs = np.zeros((4 , 1))

        ###--- Objetos ---###
        self.img_flag = False
        self.pos_aruco = Point()
        self.pos_camera = Point()
        self.bd_object = CvBridge()
        self.aruco_dictionary = cv2.aruco.Dictionary.get(cv2.aruco.DICT_6x6_250)
        self.aruco_detector = cv2.aruco.ArucoDetector_create(self.aruco_dictionary)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        ###--- Subscriptores ---###
        rospy.Subscriber("/camera/image_raw" , Image , self.camera_cb)

        ###--- Publishers ---###
        self.aruco_flag_pub = rospy.Publisher("aruco_flag" , Bool , queue_size=5)
        self.pos_aruco_pub = rospy.Publisher("pos_Aruco" , Point , queue_size=10)

        while not self.img_flag: print ("Camara no conectada")

        while not rospy.is_shutdown():
            print("Camara operando")
            flag , tvecs , rvecs = self.aruco_finder()

            if flag:
                self.pos_aruco.x = tvecs[0]
                self.pos_aruco.y = tvecs[1]
                self.pos_aruco.z = tvecs[2]
                self.aruco_flag_pub.publish(flag)
                self.pos_aruco_pub.publish(self.pos_aruco)

    def aruco_finder(self , img):
        img = self.img
        corners , ids , rejected = self.aruco_detector.detectMarkers(img , parameters=self.aruco_params)
        flag = False

        if ids is not None:
            flag = True
            img = cv2.aruco.drawDetectedMarkers(img , corners , ids)

            for i in range(len(ids)):
                rvecs , tvecs , _ = cv2.aruco.estimatePoseSingleMarkers(corners[i] , 0.05 , self.test_camera)
                img = cv2.aruco.drawAxis(img , self.test_camera , self.test_coefs , rvecs , tvecs , 0.1)

                distance = tvecs[0][0][2]
                cv2.putText(img , f"Distance: {distance:.2f} meters" , (20 , 40), cv2.FONT_HERSHEY_SIMPLEX , 1 , (0 , 255 , 0) , 2)
        
        if flag:
            return flag , tvecs , rvecs
        else: return flag
        
    def camara_cb (self , data):
        try:
            self.img = self.bd_object.imgmsg_to_cv2(data , desired_encoding="bgr8")
            self.img_flag = True
        except CvBridgeError as error:
            self.img_flag = False
            print (error)

    def cleanup (self):
        print ("Apagando Camara")
        