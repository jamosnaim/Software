#!/usr/bin/env python


import rospy
# import rospkg

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

import numpy as np


class Controlador():

    def __init__(self):


        #Subscribirse al topico "/duckiebot/camera_node/image/raw"
        self.point_subscriber = rospy.Subscriber('punto', Point, self._process_image)
        self.control_subscriber = rospy.Subscriber('/duckiebot/joy', Joy,self._process_callback)
        self.motor_publisher = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.hayPato=False
        self.posicionPato=0
        

    def _process_image(self,punto):
        if punto.z==-1:
            self.hayPato=False
            self.posicionPato=0
        else:
            self.hayPato=True
            self.posicionPato=punto.z




    def _process_callback(self,control):
        msg=Twist2DStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.v=0.5*control.axes[1]
        msg.omega = 8.3*control.axes[0]
        if self.hayPato and self.posicionPato<30 and msg.v>0 and not control.buttons[0]:
            msg.v=0
        self.motor_publisher.publish(msg)
def main():

    rospy.init_node('Controlador')

    Controlador()

    rospy.spin()

if __name__ == '__main__':
    main()
