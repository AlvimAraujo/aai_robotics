#!/usr/bin/env python
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement, RosiMovementArray
from geometry_msgs.msg import Twist

class RosiNodeClass():

	# class attributes
	max_translational_speed = 5 # in [m/s]
	max_rotational_speed = 10 # in [rad/s]

	# how to obtain these values? see Mandow et al. COMPLETE THIS REFERENCE
	var_lambda = 0.965
	wheel_radius = 0.1324
	ycir = 0.531

	# class constructor
	def __init__(self):

		# initializing some attributes
		self.omega_left = 0
		self.omega_right = 0

		# computing the kinematic A matrix
		self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

		# sends a message to the user
		rospy.loginfo('Rosi_joy node started')

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)

		# registering to subscribers
		self.sub_cmd_vel = rospy.Subscriber('/aai_rosi_cmd_vel', Twist, self.callback_cmd_vel)

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		# eternal loop (until second order)
		while not rospy.is_shutdown():

			traction_command_list = RosiMovementArray()

			# mounting the lists
			for i in range(4):

				# ----- treating the traction commands
				traction_command = RosiMovement()

				# mount traction command list
				traction_command.nodeID = i+1

				# separates each traction side command
				if i < 2:
					traction_command.joint_var = self.omega_right
				else:
					traction_command.joint_var = self.omega_left

				# appending the command to the list
				traction_command_list.movement_array.append(traction_command)

			# publishing
			self.pub_traction.publish(traction_command_list)

			# sleeps for a while
			node_sleep_rate.sleep()

	# joystick callback function
	def callback_cmd_vel(self, data):
		K = 10

		vel_linear_x = K*data.linear.x
		vel_angular_z = K*data.angular.z
		
		# # LIMITACAO
		# if vel_linear_x > self.max_translational_speed:
		# 	vel_linear_x = self.max_translational_speed
		#
		# if vel_linear_x < -self.max_translational_speed:
		# 	vel_linear_x = -self.max_translational_speed
		#
		# if vel_angular_z > self.max_rotational_speed:
		# 	vel_angular_z = self.max_rotational_speed
		#
		# if vel_angular_z < -self.max_rotational_speed:
		# 	vel_angular_z = -self.max_rotational_speed

		# -- computes traction command - kinematic math

		# b matrix
		b = np.array([[vel_linear_x],[vel_angular_z]])

		# finds the joints control
		x = np.linalg.lstsq(self.kin_matrix_A, b, rcond=-1)[0]

		# query the sides velocities
		self.omega_right = np.deg2rad(x[0][0])
		self.omega_left = np.deg2rad(x[1][0])

	# ---- Support Methods --------

	# -- Method for compute the skid-steer A kinematic matrix
	@staticmethod
	def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):

		# kinematic A matrix
		matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
							[(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])

		return matrix_A

# instaciate the node
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('rosi_example_node', anonymous=True)

	# instantiate the class
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass
