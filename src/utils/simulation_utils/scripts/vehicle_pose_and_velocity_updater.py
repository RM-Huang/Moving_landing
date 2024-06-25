#!/usr/bin/env python

import sys
sys.path.append('/home/pc205/.local/lib/python3.8/site-packages')
import os
import numpy as np
from math import cos, sin, atan2, atan

from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion

from nav_msgs.msg import Odometry

from gazebo_msgs.msg import ModelStates, LinkStates

from std_msgs.msg import Float64

import tf
import rospy

class vehicle_pose_and_velocity_updater:
	def __init__(self):
		rospy.init_node('vehicle_pose_and_velocity_updater', log_level=rospy.DEBUG)

		self.yaw_from_vel = Float64()
		self.yaw_from_vel.data = 0.0

		# self.rear_pose_pub = rospy.Publisher('/smart/rear_pose', PoseStamped, queue_size = 1)
		# self.center_pose_pub = rospy.Publisher('/smart/center_pose', PoseStamped, queue_size = 1)
		# self.vel_pub = rospy.Publisher('/smart/velocity', TwistStamped, queue_size = 1)
		# self.yaw_from_vel_pub = rospy.Publisher('/debug/yaw_vel', Float64, queue_size = 1)
		# self.yaw_from_truth_pub = rospy.Publisher('/debug/yaw_tru', Float64, queue_size = 1)
		# self.qua_from_vel_pub = rospy.Publisher('/debug/qua_vel', Quaternion, queue_size = 1)
		self.odom_pub = rospy.Publisher('/smart/odom', Odometry, queue_size=1)

		rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size = 1)

		rospy.spin()

	def model_cb(self,data):
		try:
			vehicle_model_index = data.name.index("smart")
		except:
			return
		vehicle_position = data.pose[vehicle_model_index]
		vehicle_velocity = data.twist[vehicle_model_index]
		orientation = vehicle_position.orientation
		(_, _, yaw) = tf.transformations.euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
		time_stamp = rospy.Time.now()

		# vehicle center position
		center_pose = PoseStamped()
		center_pose.header.frame_id = '/world'
		center_pose.header.stamp = time_stamp
		center_pose.pose.position = vehicle_position.position
		center_pose.pose.orientation = vehicle_position.orientation
		# self.center_pose_pub.publish(center_pose)

		# vehicle rear axle position
		rear_pose = PoseStamped()
		rear_pose.header.frame_id = '/world'
		rear_pose.header.stamp = time_stamp
		center_x = vehicle_position.position.x
		center_y = vehicle_position.position.y
		rear_x = center_x - cos(yaw) * 0.945
		rear_y = center_y - sin(yaw) * 0.945
		rear_pose.pose.position.x = rear_x
		rear_pose.pose.position.y = rear_y
		rear_pose.pose.orientation = vehicle_position.orientation
		# self.rear_pose_pub.publish(rear_pose)

		# vehicle velocity
		velocity = TwistStamped()
		velocity.header.frame_id = ''
		velocity.header.stamp = time_stamp
		velocity.twist.linear = vehicle_velocity.linear
		velocity.twist.angular = vehicle_velocity.angular
		# self.vel_pub.publish(velocity)

		# get orientation from velocity
		singul = 1
		if(vehicle_velocity.linear.x < 0):
			singul = -1
		# yaw_from_vel = Float64()
		yaw_from_truth = Float64()
		qua_from_vel_msg = Quaternion()
		# yaw_from_vel.data = atan2(singul * vehicle_velocity.linear.y, singul * vehicle_velocity.linear.x)
		if(vehicle_velocity.linear.x > 0 and vehicle_velocity.linear.y >= 0):
			self.yaw_from_vel.data = atan(vehicle_velocity.linear.y / vehicle_velocity.linear.x)
		elif(vehicle_velocity.linear.x > 0 and vehicle_velocity.linear.y < 0):
			self.yaw_from_vel.data = 2 * 3.14159265 - atan(- vehicle_velocity.linear.y / vehicle_velocity.linear.x)
		elif(vehicle_velocity.linear.x < 0 and vehicle_velocity.linear.y >= 0):
			self.yaw_from_vel.data = 3.14159265 - atan(vehicle_velocity.linear.y / - vehicle_velocity.linear.x)
		elif(vehicle_velocity.linear.x < 0 and vehicle_velocity.linear.y < 0):
			self.yaw_from_vel.data = atan(- vehicle_velocity.linear.y / - vehicle_velocity.linear.x) + 3.14159265
		elif(vehicle_velocity.linear.x == 0 and vehicle_velocity.linear.y > 0):
			self.yaw_from_vel.data = 1.570796325
		elif(vehicle_velocity.linear.x == 0 and vehicle_velocity.linear.y < 0):
			self.yaw_from_vel.data = -1.570796325
		
		yaw_from_truth.data = yaw
		qua_from_vel = tf.transformations.quaternion_from_euler(0,0,self.yaw_from_vel.data)
		qua_from_vel_msg.w = qua_from_vel[3]
		qua_from_vel_msg.x = qua_from_vel[0]
		qua_from_vel_msg.y = qua_from_vel[1]
		qua_from_vel_msg.z = qua_from_vel[2]
		# self.yaw_from_vel_pub.publish(yaw_from_vel)
		# self.yaw_from_truth_pub.publish(yaw_from_truth)
		# self.qua_from_vel_pub.publish(qua_from_vel_msg)

		# odometry
		odometry = Odometry()
		odometry.header.stamp = time_stamp
		odometry.pose.pose.position = vehicle_position.position
		odometry.pose.pose.orientation = qua_from_vel_msg
		odometry.twist.twist.linear = vehicle_velocity.linear
		self.odom_pub.publish(odometry)


if __name__ == "__main__":
	try:
		vehicle_pose_and_velocity_updater()
	except:
		rospy.logwarn("cannot start vehicle pose and velocity updater updater")