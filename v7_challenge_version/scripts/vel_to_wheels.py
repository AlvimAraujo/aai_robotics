#!/usr/bin/env python
######################################################################
# CODIGO QUE CONVERTE SINAIS DE VELOCIDADE EM COMANDOS PARA AS RODAS #
######################################################################
# Este no representa o controle de baixo nivel

import rospy
# Tipos de mensagens utilizadas
from rosi_defy.msg import RosiMovement, RosiMovementArray
from geometry_msgs.msg import Twist

# Classe RosiNodeClass responsavel por todo o processo do programa
class RosiNodeClass():

	# Atributos segundo o manual
	max_translational_speed = 5 # in [m/s]
	max_rotational_speed = 10 # in [rad/s]
	var_lambda = 0.965
	wheel_radius = 0.1324
	ycir = 0.531

	#Distancia entre as rodas direitas e esquerdas
	L = 0.6

	# Construtor
	def __init__(self):

		# Comandos que serao enviados para as rodas da direita e da esquerda
		self.omega_left = 0
		self.omega_right = 0

		# Atalhos para informar que deve mandar velocidade 0 quando nao houver comandos de velocidade
		self.last = 0
		self.TIME_OUT = 0.1

		# Mensagem de inicializacao
		rospy.loginfo('Controle de baixo nivel iniciado')

		# Publicar em command_traction_speed
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
		# Subscrever em aai_rosi_cmd_vel
		self.sub_cmd_vel = rospy.Subscriber('/aai_rosi_cmd_vel', Twist, self.callback_cmd_vel)
		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(10)

		# Loop principal, responsavel pelos procedimentos chaves do programa
		while not rospy.is_shutdown():

			# Comando de tracao a ser publicado
			traction_command_list = RosiMovementArray()

			# Criar traction_command_list como uma soma de traction_commands
			for i in range(4):
				# Um comando de tracao por roda
				traction_command = RosiMovement()
				# ID da roda
				traction_command.nodeID = i+1
				# Publicar 0 caso nao haja comandos de velociade
				if rospy.get_rostime().to_sec() - self.last >= self.TIME_OUT:
					traction_command.joint_var = 0
				else:
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

	# Callback da leitura do cmd_vel
	def callback_cmd_vel(self, msg):

		self.omega_right = (msg.linear.x + (self.L/2)*msg.angular.z)/self.wheel_radius
		self.omega_left = (msg.linear.x - (self.L/2)*msg.angular.z)/self.wheel_radius

		self.last = rospy.get_rostime().to_sec()

# Funcao principal
if __name__ == '__main__':

	# Inicializa o no
	rospy.init_node('rosi_vel_to_wheels_node', anonymous=True)

	# Inicializa o objeto
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException:
		pass
