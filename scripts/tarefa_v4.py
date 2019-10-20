#!/usr/bin/env python
#################################
# CODIGO DE CONTROLE PRINCIAPAL #
#################################
# Este no calcula os sinais de velocidade para o robo e tambem
# indica quando devem ser feitas as tarefas de toque e de escada

from __future__ import print_function # apenas para imprimir os erros, caso existam e debuga-los
# Pacotes
import rospy
import roslib
import sys
import cv2
import numpy as np
# Tipos de mensagens utilizadas
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
from sensor_msgs.msg import Image
# Ferramentas para o processamento dos dados
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, sqrt

# Classe que contem os metodos necessarios para o programa
class RosiCmdVelClass():

	# Constantes de controle
	Kp = 0.5 # Ganho proporcional
	d = 0.1 # Distancia entre o centro de massa e o ponto a ser controlado por Feedback Linearization
	Err = 0.5 # Erro admitido de distancia ao ponto

	# Construtor
	def __init__(self):
		# Posicao (x, y, theta)
		self.pos_x = 0.1
		self.pos_y = 0.1
		self.angle = 0.1
		# Posicao relativa do ponto a ser desviado (obstaculo)
		self.xd = 10
		self.yd = 10
		# Variavel de controle
		self.state = 0
		# Tratamento e conversao de imagem
		self.bridge = CvBridge()

		# Topicos que subscreve e publica
		self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
		self.pub_cmd_vel = rospy.Publisher('/aai_rosi_cmd_vel', Twist, queue_size=1)
		self.image_sub = rospy.Subscriber('/aai_depth_show',Image,self.callback_image)
		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(10)

		# Mensagem de inicializacao
		rospy.loginfo('Controle de alto nivel iniciado: Campos Potenciais Artificias')

		Pontos = list()

		# Loop principal que manda as velocidades para o robo ate que ele chegue nas proximidades do ponto
		while not rospy.is_shutdown():

			Pontos = [(0, 2.5), (-15, 2.5), (-30, 2.5), (-35, 2.5)]

			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:
					try:
						[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

						vel_msg.linear.x = V
						vel_msg.angular.z = W
					except:
						vel_msg.linear.x = 0
						vel_msg.angular.z = 0

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			# Tarefa da escada
			self.state = 1
			self.Err = 0.2
			self.Kp = 0.3
			Pontos = [(-38, 2), (-41.5, 1.8)]

			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					try:
						[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

						vel_msg.linear.x = V
						vel_msg.angular.z = W
					except:
						vel_msg.linear.x = 0
						vel_msg.angular.z = 0

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			# Plataforma suspensa
			self.state = 1
			Pontos = [(-43, 1.9), (-45, 1.85), (-48, 1.85)]

			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					try:
						[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

						vel_msg.linear.x = V
						vel_msg.angular.z = W
					except:
						vel_msg.linear.x = 0
						vel_msg.angular.z = 0

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			# Voltando na plataforma
			self.state = 2
			self.d *= -1 # Robo sera controlado de costas
			Pontos = [(-45, 1.85), (-43, 1.85), (-41, 1.85), (-39, 2.5)]

			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					try:
						[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

						vel_msg.linear.x = V
						vel_msg.angular.z = W
					except:
						vel_msg.linear.x = 0
						vel_msg.angular.z = 0

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			# Dando uma volta no TC
			self.d *= -1 # De frente
			self.state = 0
			self.Err = 0.5
			self.Kp = 0.5
			Pontos = [(-40, 4), (-55, 3), (-55, -4), (-40, -3.5), (-30, -3.5), (-15, -3.5), (0, -3.5), (0, 3)]

			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					try:
						[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

						vel_msg.linear.x = V
						vel_msg.angular.z = W
					except:
						vel_msg.linear.x = 0
						vel_msg.angular.z = 0

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			# Tarefa de toque
			self.state = 2
			self.Err = 0.2
			self.Kp = 0.3
			Pontos = [(-3, 2.5), (-4, 2), (-6.987, 1.8)]

			for (x_goal, y_goal) in Pontos:
				vel_msg = Twist()
				while abs(self.pos_x - x_goal) > self.Err or abs(self.pos_y - y_goal) > self.Err:

					try:
						[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

						vel_msg.linear.x = V
						vel_msg.angular.z = W
					except:
						vel_msg.linear.x = 0
						vel_msg.angular.z = 0

					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			self.pub_cmd_vel.publish(vel_msg)

			rospy.set_param('touch_mode', True)

			# # Esperar ate que o toque tenha terminado
			# while rospy.get_param('touch_mode'):
			# 	pass

			# Este programa terminou sua rotina
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			self.pub_cmd_vel.publish(vel_msg)
			foo() # :)
			break


	def calc_vel_from_potential(self, current_x, current_y, current_theta, x_goal, y_goal):
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
			Ka = 1
			d_max = 1.5

			Kr = 1
			d_min = 1.5

		# Campo potencial atrativo
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
		self.angle = euler_angles[2] # Apenas o angulo de Euler no eixo z nos interessa


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

# Mensagem de despedida e finalizacao
def foo():
	print('\n'*5)
	print('A execucao chegou ao fim.\n')
	print('A equipe AAI Robotics agradece pela oportunidade!')
	print('Foi desafiador para todos os tres membros do grupo.')
	print('Aprendemos muito com o desafio e estamos orgulhosos de termos chegado ate aqui.\n')
	print('Foi um prazer!\n')
	print('Atenciosamente,\n')
	print('Alvaro Rodrigues Araujo')
	print('Arthur Henrique Dias Nunes')
	print('Israel Filipe Silva Amaral')
	print('\nAAI Robotics - Universidade Federal de Minas Gerais')

# Funcao main
if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiCmdVelClass()
    except rospy.ROSInterruptException:
        pass
