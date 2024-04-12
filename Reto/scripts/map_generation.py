import rospy 
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point


class motor_control:
    def __init__(self):
        ###--- Inicio del Nodo ---###
        rospy.init_node('motor_control')
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(30)
        #self.camera = cv2.VideoCapture(0)

        ###--- Parametros ---###
        self.test_camera= np.array([[1000 , 0 , 320] , [0 , 1000 , 240] , [0 , 0 , 1]])
        

        ###--- Objetos ---###
        self.img_flag = False

        ###--- Subscriptores ---###
        rospy.Subscriber("/camera/image_raw" , Image , self.camera_cb)

        ###--- Publishers ---###
        self.pos_aruco_pub = rospy.Publisher("pos_Aruco" , Point , queue_size=10)
        
        ###--- Security while ---###
        while not self.img_flag: print ("Camara no conectada")

        while not rospy.is_shutdown():
            pass

        
    def camara_cb (self , data):
        pass

    def cleanup (self):
        print ("Apagando Camara")
        

if __name__ == "__main__": 
    motor_control()