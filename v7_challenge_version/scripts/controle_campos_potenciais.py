#!/usr/bin/env python
######################################################################
# CODIGO DE CONTROLE: DADA A POSICAO CALCULA OS SINAIS DE VELOCIDADE #
######################################################################
import rospy
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, sqrt
from rosi_defy.msg import HokuyoReading

# Constantes de controle
# Ganho proporcional
Kp = 1
# Distancia entre o centro de massa e a ponta
d = 0.1
# Precisao na chegada ao ponto
Err = 0.5

MAX_VEL = 1.5

MIN_VEL = 0.3

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
		self.pos_z = 0.1
		self.angle = 0.1

		# Posicao do obstaculo mais proximo
		self.nearest_obstacle_x = 0.1
		self.nearest_obstacle_y = 0.1

		# Nos que subscreve e publica 
		self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
		self.sub_hok = rospy.Subscriber('/aai_rosi_border_obstacle', HokuyoReading, self.callback_hok)
		self.pub_cmd_vel = rospy.Publisher('/aai_rosi_cmd_vel', Twist, queue_size=1)

		# Frequencia de publicacao
		node_sleep_rate = rospy.Rate(10)
		
		# Mensagem de inicializacao
		rospy.loginfo('campo potencial iniciado')

		# Loop principal, responsavel pelos procedimentos chaves do programa
		while not rospy.is_shutdown():
			x_goal_str, y_goal_str = raw_input('Digite a coordenada (x,y) desejada: ').split()
			x_goal, y_goal = [float(i) for i in [x_goal_str, y_goal_str]]
			vel_msg = Twist()
			while abs(self.pos_x - x_goal) > Err or abs(self.pos_y - y_goal) > Err:

				[V, W] = self.calc_vel_from_potential(self.pos_x, self.pos_y, self.angle, x_goal, y_goal)

				vel_msg.linear.x = V
				vel_msg.angular.z = W

				self.pub_cmd_vel.publish(vel_msg)
				node_sleep_rate.sleep()


	def calc_vel_from_potential(self, current_x, current_y, current_theta, x_goal, y_goal):
		# Campo potencial atrativo
		Ka = 0.1
		d_max = 5
		Pf_q = sqrt( (x_goal - current_x)**2 + (y_goal - current_y)**2 )

		if Pf_q <= d_max:
			vel_x_att = Ka * (x_goal - current_x)
			vel_y_att = Ka * (y_goal - current_y)
		else:
			vel_x_att = d_max* Ka * (x_goal - current_x)/Pf_q
			vel_y_att = d_max* Ka * (y_goal - current_y)/Pf_q


		# Campo potencial repulsivo
		# (b_x, b_y) eh o ponto de algum obstaculo que esta mais proximo do robo
		b_x = self.nearest_obstacle_x
		b_y = self.nearest_obstacle_y

		# print('Nearest: ', b_x, b_y)
		Kr = 0.2
		d_min = 3
		P_q = sqrt( (current_x - b_x)**2 + (current_y - b_y)**2 )

		if P_q <= d_min:
			vel_x_rep = Kr * ( (1/P_q) - (1/d_min) ) * (1/P_q**2) *  (current_x - b_x)
			vel_y_rep = Kr * ( (1/P_q) - (1/d_min) ) * (1/P_q**2) *  (current_y - b_y)
		else:
			vel_x_rep = 0
			vel_y_rep = 0

		# Campo potencial final
		vel_x = vel_x_att + vel_x_rep
		vel_y = vel_y_att + vel_y_rep

		# Feedback Linearization
		V_forward = cos(current_theta) * vel_x + sin(current_theta) * vel_y
		W_angular = (-sin(current_theta) / d) * vel_x + (cos(current_theta) / d) * vel_y

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

	def callback_hok(self, data):

		borda_x = data.reading[0::3]
		borda_y = data.reading[1::3]
		borda_z = data.reading[2::3]

		try:
			# Se existir dados do Hokuyo
			min = sqrt( (self.pos_x - borda_x[0])**2 + (self.pos_y - borda_y[0])**2 )
			for i in range(len(borda_x)):
				if min > sqrt( (self.pos_x - borda_x[i])**2 + (self.pos_y - borda_y[i])**2 ):
					min = sqrt( (self.pos_x - borda_x[i])**2 + (self.pos_y - borda_y[i])**2 )
					self.nearest_obstacle_x = borda_x[i]
					self.nearest_obstacle_y = borda_y[i]
		except:
			# Senao
			min = 0
			self.nearest_obstacle_x = self.pos_x + 100
			self.nearest_obstacle_y = self.pos_y + 100


# Funcao main
if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiCmdVelClass()
    except rospy.ROSInterruptException:
        pass
