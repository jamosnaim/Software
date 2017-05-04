#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)	
cliente = rospy.ServiceProxy('take_image',Empty)
def main():
	rospy.init_node('actividad1')
	rospy.loginfo('actividad1')
	sub = rospy.Subscriber('/duckiebot/joy', Joy,process_callback)
	rospy.spin()


def process_callback(msg):
	global base_pub
	v=0	
	if msg.buttons[2]:
		v=0
	elif msg.buttons[3]:
		v= -0.5
	elif msg.buttons[0]:
		v = 0.5*msg.axes[1]
	elif msg.buttons[1]:
		v=3*msg.axes[1]
	if msg.buttons[5]:
		resp=cliente()
	msg2=Twist2DStamped()
	msg2.header.stamp = rospy.get_rostime()
	msg2.omega = 8.3*msg.axes[0]
	msg2.v = v
	base_pub.publish(msg2)


if __name__ == '__main__':
    main()

