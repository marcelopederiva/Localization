#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import rospy
import math

localization = PoseWithCovarianceStamped()

class RobotState:
	def __init__(self):
		self.bla = True
		self.lat_inicial = 0.0
		self.lon_inicial = 0.0

	def callback(self, measures):
		if self.bla:
	 		self.lat_inicial = measures.latitude*10000
			self.lon_inicial = measures.longitude*10000
			self.bla = False	

		x = ((measures.latitude*10000 - self.lat_inicial)/60)*1852
		y = ((measures.longitude*10000 - self.lon_inicial)/60)*1852
		# print('hi')
		localization.pose.pose.position.x = x
		localization.pose.pose.position.y = y
		localization.header.frame_id = 'base_link'	
		print('{0:.16f}'.format(localization.pose.pose.position.x))


def convert():


	publish_rate = 400.0
	rospy.init_node('convert_lon_lat_node', anonymous=True)
  
	convert_pub = rospy.Publisher('/gps_position',PoseWithCovarianceStamped,queue_size = 10)
	
	convert_sub = rospy.Subscriber('/xsens/gps_data',NavSatFix,RobotState().callback)
                                  
	rate = rospy.Rate(publish_rate) 

	while not rospy.is_shutdown():
		convert_pub.publish(localization)
		rate.sleep()

if __name__ == '__main__':
	try:
		convert()
  	except rospy.ROSInterruptException:
		pass
