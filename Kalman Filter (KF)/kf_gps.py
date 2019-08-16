#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import rospy
import numpy as np 
import math
import matplotlib.pyplot as plt 

x_gps = np.zeros((2, 1))
# Estimation parameter of EKF
Q = np.diag([0.4, 0.4])**2  # predict state covariance
R = np.diag([10, 10])**2  # Observation x,y position covariance

odom = Odometry()
localization = PoseWithCovarianceStamped()
def gps_position(ms_gps):
	x_gps[0,0] = ms_gps.pose.pose.position.x
	x_gps[1,0] = ms_gps.pose.pose.position.y

def ekf(xEst,PEst,z): 

	############# PREDICTION STEP ################
	t = 0
	t = time.time()
	xPred = xEst 	
	PPred = PEst +Q

	################ UPDATE STEP #################

	S = PPred+R

	K = np.matmul(PPred,np.linalg.inv(S))

	y = z - xPred
	xEst = xPred +np.matmul(K,y)
	PEst = np.matmul(PPred,(np.eye(len(xEst)) - K))

	############## KF END ######################

	X = xEst[0,0] - xPred[0,0]
	Y = xEst[1,0] - xPred[1,0]
	deltax = math.sqrt(X**2+Y**2)
	Dt = time.time() - t


	odom.twist.twist.linear.x = deltax/Dt
	localization.pose.pose.position.x = xEst[0,0]
	localization.pose.pose.position.y = xEst[1,0]

	return xEst,PEst	

def main():


	# State Vector [x y]'
	xEst = np.zeros((2, 1))
	PEst = np.diag([10, 10])

	publish_rate = 30
	rospy.init_node('ekf_sensor', anonymous=True)

	ekf_pub = rospy.Publisher('/kf_gps',PoseWithCovarianceStamped,queue_size = 10)
	# odom_pub = rospy.Publisher('/ekf_odom',Odometry,queue_size = 10)

	gps_sub = rospy.Subscriber('/gps_position',PoseWithCovarianceStamped,gps_position)

	                              
	rate = rospy.Rate(publish_rate) 

	while not rospy.is_shutdown():

		z = x_gps

		xEst, PEst = ekf(xEst, PEst, z)

		ekf_pub.publish(localization)
		# odom_pub.publish(odom)

		print(localization)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

