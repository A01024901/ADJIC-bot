#!/usr/bin/env python3 

import rospy 
import cv2
import numpy 

class camara:
    def __init__(self):
        self.camara = cv2.VideoCapture(0)
        