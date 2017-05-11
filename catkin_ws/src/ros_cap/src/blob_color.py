#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,0,0])
upper_red = np.array([0,0,0])
lower_yellow = np.array([40*255/360,100,70])
upper_yellow = np.array([100*255/360,255,255])


class BlobColor():

    def __init__(self):


        #Subscribirse al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('/duckiebot/camera_node/image/raw', Image, self._process_image)
        self.image_publisher = rospy.Publisher('la/imagen',Image, queue_size=1)
        self.image_pubm = rospy.Publisher('la/imagen_mascarada',Image, queue_size=1)
        self.point_publisher = rospy.Publisher('punto',Point, queue_size=1)

        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 100

        self.altura_pato = 3.5



    def _process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)
        imagen_mascarada = self.bridge.cv2_to_imgmsg(segment_image, "bgr8")
        self.image_pubm.publish(imagen_mascarada)

        kernel1 = np.ones((3,3),np.uint8)
        kernel2 = np.ones((5,5),np.uint8)
        #Operacion morfologica erode
        mask = cv2.erode(mask, kernel1, iterations = 1)
        
        #Operacion morfologica dilate
        mask = cv2.dilate(mask, kernel2, iterations = 3)

        mask, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        punto=Point()
        punto.z=0
        area_max=0
        altura_imagen=0
        for cnt in contours:
            #Obtener rectangulo
            x,y,w,h = cv2.boundingRect(cnt)

            #Filtrar por area minima
            if w*h > self.min_area:

                #Dibujar un rectangulo en la imagen
                cv2.rectangle(frame, (x,y), (x+w,y+h), (255,128,0), 2)
                if w*h>area_max: 
                    area_max=w*h
                    punto.x=x+w/2
                    punto.y=y+h/2
                    altura_imagen=h
        
        f=348.2990501038053  
        if altura_imagen=0: 
            punto.z=-1
        else:  
            punto.z=f*self.altura_pato/altura_imagen
        print punto
        #Publicar frame
        imagen_final = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_publisher.publish(imagen_final)
        #Publicar Point center de mayor tamanio
        self.point_publisher.publish(punto)

def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
