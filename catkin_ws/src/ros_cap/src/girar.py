#!/usr/bin/env python


import rospy
# import rospkg

from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

import numpy as np


class Girar():

    def __init__(self):


        #Subscribirse al topico "/duckiebot/camera_node/image/raw"
        self.point_subscriber = rospy.Subscriber('punto', Point, self._process_image)
        self.motor_publisher = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)






    def _process_image(self,punto):
        msg=Twist2DStamped()
        msg.header.stamp = rospy.get_rostime()
        if punto.x>330:
            
	        msg.omega = -7
        elif punto.x<310:
            msg.omega = 7

        self.motor_publisher.publish(msg)
def main():

    rospy.init_node('Girar')

    Girar()

    rospy.spin()

if __name__ == '__main__':
    main()
