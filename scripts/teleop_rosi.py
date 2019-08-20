#!/usr/bin/env python
# CODIGO QUE CONTROLA ROSI PELO TECLADO #
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from geometry_msgs.msg import Twist

#Classe RosiNodeClass responsavel por todo o processo do programa
class RosiNodeClass():

	# Atributos segundo o manual
	max_translational_speed = 5 # in [m/s]
	max_rotational_speed = 10 # in [rad/s]
	var_lambda = 0.965
	wheel_radius = 0.1324
	ycir = 0.531

	# Class constructor
	def __init__(self):

		# Comandos que serao enviados para as rodas da direita e da esquerda
		self.omega_left = 0
		self.omega_right = 0

		# Mensagem de inicializacao
		rospy.loginfo('Rosi_teleop node started')

		# Publicar no no command_traction_speed
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)

		# Subscrever no no cmd_vel
		self.sub_teleop = rospy.Subscriber('/cmd_vel', Twist, self.callback_teleop)

		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(10)

		# Loop principal, responsavel pelos procedimentos chaves do programa
		while not rospy.is_shutdown():

			# Comando de tracao final
			traction_command_list = RosiMovementArray()

			# Criar traction_command_list
			for i in range(4):

				# Um comando de tracao por roda
				traction_command = RosiMovement()

				# ID da roda
				traction_command.nodeID = i+1

				# Separa as rodas do lado direito do esquerdo
				if i < 2:
					traction_command.joint_var = self.omega_right
				else:
					traction_command.joint_var = self.omega_left

				# Adiciona o comando ao comando de tracao final
				traction_command_list.movement_array.append(traction_command)

			# Publicacao
			self.pub_traction.publish(traction_command_list)

			# Pausa
			node_sleep_rate.sleep()

	# Callback da leitura do teclado (teleop callback)
	def callback_teleop(self, msg):
		self.omega_right = 5*msg.linear.x + 5*msg.angular.z
		self.omega_left = 5*msg.linear.x - 5*msg.angular.z

# Funcao principal
if __name__ == '__main__':

	# Inicializa o no
	rospy.init_node('rosi_teleop_node', anonymous=True)

	# Inicializa o objeto
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass
