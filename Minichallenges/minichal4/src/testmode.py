#!/usr/bin/env python3 

import rospy 
import numpy as np
import os
import time
import pandas as pd
from geometry_msgs.msg import Twist

class test_mode:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('test_mode')
        rospy.on_shutdown(self.cleanup)

        ###--- Publishers ---###
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)

        ###--- Constants ---###
        self.dt = 0.02

        ###--- Path ---###
        ruta = os.path.dirname(os.path.abspath(__file__))

        ###--- Objetos ---####
        linear = pd.read_csv(ruta + '/experimentation/exp_lineares.csv')
        self.velocidades_l = linear["Velocidad"]
        self.duraciones_l = linear["Tiempo"]
        angulares = pd.read_csv(ruta + '/experimentation/exp_angulares.csv')
        #Cambiar el nombre del topico a Grados en vez de R/S
        self.velocidades_a = angulares["R/S"]
        self.duraciones_a = angulares["Tiempo"]

        rate = rospy.Rate(int(1.0/self.dt))

        self.cmd_vel = Twist()

        #while rospy.get_time() == 0: print ("Simulacion no iniciada")#Descomentar en simulacion 

        print("Simulacion operando")

        while not rospy.is_shutdown():
            
            self.testmode()
            
            rate.sleep()

    def testmode(self):
        test = (input("Deseas probar velocidad angular o lineal: "))
        exp_num = np.random.randint(0, 50)
        if ("a" in test.lower() and "g" in test.lower()):
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = np.deg2rad(float(self.velocidades_a[exp_num]))
            self.cmd_vel_pub.publish(self.cmd_vel)
            time.sleep(float(self.duraciones_a[exp_num]))
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd_vel)
            print("\n")

        elif ("e" in test.lower() and "i" in test.lower()):
            self.cmd_vel.linear.x = float(self.velocidades_l[exp_num])
            self.cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd_vel)
            time.sleep(float(self.duraciones_l[exp_num]))
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd_vel)
            print("\n")

    def cleanup(self):
        print ("Apagando Simulacion")
        clean = Twist()
        self.cmd_vel_pub.publish(clean)
        

if __name__ == "__main__": 
    test_mode()