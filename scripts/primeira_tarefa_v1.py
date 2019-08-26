#!/usr/bin/env python
######################################################################
# CODIGO DE CONTROLE: DADA A POSICAO CALCULA OS SINAIS DE VELOCIDADE #
######################################################################
import rospy
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos

# Constantes de controle, necessita calibrar
# Ganhor proporcional
Kp = 1
# Distancia entre o centro de massa e a ponta
d = 0.4
# Precisao na chegada ao ponto
Err = 0.5

# Funcao que calcula os sinais V e omega dado Vx, Vy, e a posicoes inicial e final
def calc_vel_feedback_linearization(current_x, current_y, current_theta, x_goal, y_goal):
	#LEMBRETE: IMPLEMENTAR LIMITACAO DAS VELOCIDADES
	vel_x = Kp * (x_goal - current_x)
	vel_y = Kp * (y_goal - current_y)

	V_forward = cos(current_theta) * vel_x + sin(current_theta) * vel_y
	W_angular = (-sin(current_theta) / d) * vel_x + (cos(current_theta) / d) * vel_y

	return (V_forward, W_angular)

# Classe que contem os metodos necessarios para o programa
class RosiCmdVelClass():

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

		# Nos que subscreve e publica # kkkk
		self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
		self.pub_cmd_vel = rospy.Publisher('/aai_rosi_cmd_vel', Twist, queue_size=1)

		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(10)
		# Mensagem de inicializacao
		rospy.loginfo('primeira_tarefa_v1 iniciado')

		# Loop principal, responsavel pelos procedimentos chaves do programa
		while not rospy.is_shutdown():
			#x_goal_str, y_goal_str = raw_input('Digite a coordenada (x,y) desejada: ').split()
			#x_goal, y_goal = [float(i) for i in [x_goal_str, y_goal_str]]

			Pontos = list()
			for i in range(41):
				Pontos += [(-i, 2.5)]

			for (x_goal, y_goal) in Pontos:
				while abs(self.pos_x - x_goal) > Err or abs(self.pos_y - y_goal) > Err:
					vel_msg = Twist()
					[vel_msg.linear.x, vel_msg.angular.z] = calc_vel_feedback_linearization(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)
					self.pub_cmd_vel.publish(vel_msg)
					node_sleep_rate.sleep()

			#Pontos += [(-i-2, 3)]
			Pontos.reverse()




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

# Funcao main
if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiCmdVelClass()
    except rospy.ROSInterruptException:
        pass
