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


class clock: 
	DT = 0.0
clk = clock()

x_gps = np.zeros((2, 1))
x_input = np.zeros((2,1))
# Estimation parameter of EKF
R = np.diag([0.1,0.1,0.05,0.005])**2  # predict state covariance
# R = np.diag([0.001,0.001,0.5,0.01])**2  # predict state covariance

Q = np.diag([10,10])**2  # Observation sensor quality 

localization = PoseWithCovarianceStamped()

def input_odom(ms_odom):
	x_input[0,0] = ms_odom.twist.twist.linear.x  # [m/s]
	

def input_imu(ms_imu):
	x_input[1,0] = ms_imu.angular_velocity.z  # [rad/s]
	
def gps_position(ms_gps):
	x_gps[0,0] = ms_gps.pose.pose.position.x
	x_gps[1,0] = ms_gps.pose.pose.position.y




def motion(x,u,DT):

	A = np.array([[1.0, 0, 0, 0],
	              [0, 1.0, 0, 0],
	              [0, 0, 0.0, 0],
	              [0, 0, 0, 1.0]])	

	B = np.array([[DT * math.cos(x[2,0]), 0],
	              [DT * math.sin(x[2,0]), 0],
	              [1.0, 0.0],
              	  [0.0, DT]])
	a = np.matmul(A,x)
	
	b = np.matmul(B,u)

	x = a + b 

	return x

def jacobF(x,u,DT):

	yaw = x[2,0]
	v = u[0,0]

	jF = np.array([
	[1.0, 0.0, DT * math.cos(yaw), -DT * v * math.sin(yaw)],
	[0.0, 1.0, DT * math.sin(yaw), DT * v * math.cos(yaw) ],
	[0.0, 0.0, 1.0, 0.0],
	[0.0, 0.0, 0.0, 1.0]])

	return jF	


def jacobH(x):
	# Jacobian of Observation Model
	jH = np.array([
	    [1, 0, 0, 0],
	    [0, 1, 0, 0]
	])

	return jH


def observation(x):
	#  Observation Model
	H = np.array([
	    [1,0,0,0],
	    [0,1,0,0]
	])

	z = np.matmul(H,x)

	return z

def ekf(xEst,PEst,z,u): 

	t = time.time() 
	DT = clk.DT

	############# PREDICTION STEP ################
	
	xPred = motion(xEst,u,DT) 
	# print(xPred)
	jF = jacobF(xPred,u,DT)

	PPred = np.matmul(np.matmul(jF,PEst),jF.T) +R

	################ UPDATE STEP #################

	jH = jacobH(xPred)
	
	zPred = observation(xPred)

	y = np.subtract(z.T, zPred)

	S = np.matmul(np.matmul(jH,PPred),jH.T) +Q

	K = np.matmul(np.matmul(PPred,jH.T),np.linalg.inv(S))

	xEst = xPred +np.matmul(K,y)
	
	PEst = np.matmul((np.eye(len(xEst)) - np.matmul(K,jH)),PPred)

	localization.pose.pose.position.x = xEst[0,0]
	localization.pose.pose.position.y = xEst[1,0]
	clk.DT = time.time() - t 
	return xEst,PEst
		

def main():
		
	DT = 0.0

	# State Vector [x y]'
	xEst = np.array([[0],[0],[0],[0]])

	PEst = np.diag([0.00001,0.00001,0.00001,0.00001])

	publish_rate = 50
	rospy.init_node('ekf_sensor', anonymous=True)

	ekf_pub = rospy.Publisher('/ekf_m',PoseWithCovarianceStamped,queue_size = 10)

	gps_sub = rospy.Subscriber('/gps_position',PoseWithCovarianceStamped,gps_position)
	odom_sub = rospy.Subscriber('/odom',Odometry,input_odom)
	imu_sub = rospy.Subscriber('/xsens/imu',Imu,input_imu)
	                              

	rate = rospy.Rate(publish_rate) 

	while not rospy.is_shutdown():

		u = np.array([x_input[0],x_input[1]])

		z = np.array([x_gps[0],x_gps[1]])
		z = z.T

		xEst, PEst = ekf(xEst, PEst, z,u)
		ekf_pub.publish(localization)
		
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
