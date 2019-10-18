#!/usr/bin/env python
######################################################################
# CODIGO DE CONTROLE: DADA A POSICAO CALCULA OS SINAIS DE VELOCIDADE #
######################################################################
from __future__ import print_function # apenas para imprimir os erros, caso hajam

import rospy
import roslib
import sys
import cv2
import numpy as np

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, sqrt

# Classe que contem os metodos necessarios para o programa
class RosiCmdVelClass():

	# Constantes de controle, necessita calibrar
	# Ganhor proporcional
	Kp = 0.5
	# Distancia entre o centro de massa e a ponta
	d = 0.1
	# Precisao na chegada ao ponto
	Err = 0.5

    # # Atributos segundo o manual
	# max_translational_speed = 5 # in [m/s]
	# max_rotational_speed = 10 # in [rad/s]
	# var_lambda = 0.965
	# wheel_radius = 0.1324
	# ycir = 0.531

	# Class constructor
	def __init__(self):
		# Posicao (x, y, theta)
		self.pos_x = 0.1
		self.pos_y = 0.1
		self.angle = 0.1
		# Posicao dos relativa do ponto a ser desviado
		self.xd = 10
		self.yd = 10

		self.state = 0

		self.bridge = CvBridge()

		# Nos que subscreve e publica # kkkk
		self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
		self.pub_cmd_vel = rospy.Publisher('/aai_rosi_cmd_vel', Twist, queue_size=1)
		self.image_sub = rospy.Subscriber("/aai_depth_show",Image,self.callback_image)

		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(10)
		# Mensagem de inicializacao
		rospy.loginfo('campo potencial iniciado')

		Pontos = list()
		Pontos = [(0, 2.5), (-15, 2.5), (-30, 2.5), (-40, 3), (-55, 3), (-55, -4), (-40, -3.5), (-30, -3.5), (-15, -3.5), (0, -3.5), (0, 2.5), (-15, 2.5), (-30, 2.5), (-35, 2.5)]

		# Loop principal em que manda as velocidades para o robo ate que ele chegue nas proximidades do ponto
		while not rospy.is_shutdown():
			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

					vel_msg.linear.x = V
					vel_msg.angular.z = W

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()


			self.state = 1
			self.Err = 0.2
			self.Kp = 0.3
			Pontos = [(-38, 2), (-41.5, 1.8)]

			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

					vel_msg.linear.x = V
					vel_msg.angular.z = W

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			self.state = 2

			Pontos = [(-43, 1.9), (-45, 1.85), (-47, 1.85)]
			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

					vel_msg.linear.x = V
					vel_msg.angular.z = W

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			self.state = 3
			self.d *= -1
			Pontos = [(-45, 1.85), (-43, 1.85), (-41, 1.85), (-39, 2.5), (-37, 3)]
			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

					vel_msg.linear.x = V
					vel_msg.angular.z = W

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()
			self.d *= -1

			break

			#Pontos.reverse()
			#self.d *= - 1


	def calc_vel_from_potential(self, current_x, current_y, current_theta, x_goal, y_goal):
		# Campo potencial atrativo
		if self.state == 0:
			Ka = 1
			d_max = 3

			Kr = 20
			d_min = 3.5
		if self.state == 1:
			Ka = 2
			d_max = 3

			Kr = 0
			d_min = 3
		if self.state == 2:
			Ka = 2
			d_max = 3

			Kr = 0
			d_min = 3

		if self.state == 3:
			Ka = 1
			d_max = 1.5

			Kr = 1
			d_min = 1.5


		Pf_q = sqrt( (x_goal - current_x)**2 + (y_goal - current_y)**2 )

		if Pf_q <= d_max:
			vel_x_att = Ka * (x_goal - current_x)
			vel_y_att = Ka * (y_goal - current_y)
		else:
			vel_x_att = d_max* Ka * (x_goal - current_x)/Pf_q
			vel_y_att = d_max* Ka * (y_goal - current_y)/Pf_q


		# Campo potencial repulsivo
		b_x = current_x + self.xd
		b_y = current_y + self.yd

		P_q = sqrt( (current_x - b_x)**2 + (current_y - b_y)**2 )

		if P_q <= d_min:
			vel_x_rep = Kr * ( (1/P_q) - (1/d_min) ) * (1/P_q**2) *  (current_x - b_x)
			vel_y_rep = Kr * ( (1/P_q) - (1/d_min) ) * (1/P_q**2) *  (current_y - b_y)
		else:
			vel_x_rep = 0
			vel_y_rep = 0

		# Campo potencial final
		vel_x = self.Kp * (vel_x_att + vel_x_rep)
		vel_y = self.Kp * (vel_y_att + vel_y_rep)

		# Feedback Linearization
		V_forward = cos(current_theta) * vel_x + sin(current_theta) * vel_y
		W_angular = (-sin(current_theta) / self.d) * vel_x + (cos(current_theta) / self.d) * vel_y

		return (V_forward, W_angular)

	# Callback da posicao
	def callback_pose(self, data):

		q_x = data.orientation.x
		q_y = data.orientation.y
		q_z = data.orientation.z
		q_w = data.orientation.w
		# Orientacao de quaternios para angulos de Euler
		euler_angles = euler_from_quaternion([q_x, q_y, q_z, q_w])

		self.pos_x  = data.position.x
		self.pos_y = data.position.y
		self.angle = euler_angles[2] # Apenas o angulo de Euler no eixo z interessa


	def callback_image(self,data):
		# try except caso hajam erros
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
			#cv_image = cv2.flip(cv_image, 1)
		except CvBridgeError as e:
			print(e)
		depth_array = np.array(cv_image, dtype=np.float32)
		cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
		# #Para poder ver a imagem depth do kinect
		#cv2.imshow("Kinect Depth", depth_array)
		#cv2.waitKey(3)

		prox = depth_array[0][0]
		linha = 0
		coluna = 640
		#colunas = list()
		for i in range(int(0.7*data.height)):
			if min(depth_array[i]) < prox:
				prox = min(depth_array[i])
				linha = i

		for i in range(data.width):
			if abs(depth_array[linha][i] - prox) < 0.001:
				#colunas.append(i)
				if abs(310 - i) < abs(310 - coluna):
					coluna = i

		# Transformacao Homogenea para converter a posicao relativa em posicao absoluta
		Kpx = 0.005
		Kvalor = 5
		xd = cos(self.angle)*(310-coluna)*Kpx - sin(self.angle)*prox*Kvalor
		yd = sin(self.angle)*(310-coluna)*Kpx + cos(self.angle)*prox*Kvalor

		self.xd = xd
		self.yd = yd

# Funcao main
if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiCmdVelClass()
    except rospy.ROSInterruptException:
        pass
