#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

import rospy
from rosi_defy.msg import ManipulatorJoints
from geometry_msgs.msg import TwistStamped, Pose
from math import pi, sqrt
from tf.transformations import euler_from_quaternion

class RosiNodeClass():

    def __init__(self):
        self.touch = 0
        self.state = 0
        # Comandos a serem enviados para as juntas
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.joint5 = 0
        self.joint6 = 0

        # Posicao desejada das juntas
        self.desired_joint1 = 0
        self.desired_joint2 = 0
        self.desired_joint3 = 0
        self.desired_joint4 = 0
        self.desired_joint5 = 0
        self.desired_joint6 = 0

        # Publica em jointsPosTargetCommand
        self.pub_jointsPos = rospy.Publisher('/ur5/jointsPosTargetCommand',  ManipulatorJoints, queue_size=1)

        # Subscreve em jointsPositionCurrentState, forceTorqueSensorOutput e aai_rosi_pose
        self.sub_joints = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_joints)
        self.sub_force = rospy.Subscriber('/ur5/forceTorqueSensorOutput', TwistStamped, self.callback_force)
        #self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)

        # Define a frequencia de publicacao
        node_sleep_rate = rospy.Rate(10)

        # Loop principal do algoritmo
        while not rospy.is_shutdown():
            #
            traj = ManipulatorJoints()
            traj.header.stamp = rospy.Time.now()
            traj.joint_variable = [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6]

            # Publica a mensagem
            self.pub_jointsPos.publish(traj)

            # Pausa
            node_sleep_rate.sleep()


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

    # Funcao de callback das juntas
    def callback_joints(self, data):
        # Erro permitido
        self.err = 0.05
        
        
        # Checa se a rotina de toque ja foi realizada uma vez
        if(self.forceFlag == 1):
            self.touch = 1

        # No ponto especifico inicia a rotina
        #if(abs(self.pos_x - 0) >= self.err and abs(self.pos_y - 0) >= self.err and abs(self.angle - 0) >= self.err and self.touch != 1):
        self.desired_joint1 = pi/2
        self.desired_joint5 = -pi/2
        if(abs(data.joint_variable[0] - pi/2) >= self.err and abs(data.joint_variable[4] + pi/2) >= self.err):
            self.state = 1
            print('1')
        if(self.state == 1):
            self.desired_joint2 = -pi/3
            self.desired_joint4 = pi/3
        if(abs(data.joint_variable[1] + pi/3) >= self.err and abs(data.joint_variable[3] - pi/3) >= self.err):
            self.state = 2
            print('2')
        if(self.state == 2):
            self.desired_joint3 = pi/3
            self.desired_joint4 = 0
        if(abs(data.joint_variable[2] - pi/3) >= self.err and abs(data.joint_variable[3] - 0) >= self.err):
            self.state = 3
            print('3')
        if(self.state == 3):
            self.desired_joint2 = 0
            self.desired_joint4 = -pi/3
        if(abs(data.joint_variable[1] - 0) >= self.err and abs(data.joint_variable[3] + pi/3) >= self.err):
            self.state = 4
            print('4')
        if(self.state == 4):
            self.desired_joint2 = pi/2
            self.desired_joint4 = -5*pi/6
        
        # Volta as juntas para a posicao inicial
        if(self.touch == 1):
            self.desired_joint2 = 0
            self.desired_joint3 = 0
            self.desired_joint4 = 0
            if(abs(data.joint_variable[1] - 0) >= self.err and abs(data.joint_variable[2] - 0) >= self.err and abs(data.joint_variable[3] - 0) >= self.err):
                self.desired_joint1 = 0
                self.desired_joint5 = 0
                self.state = 0


        if(abs(self.desired_joint1 - data.joint_variable[0]) >= self.err):
            self.joint1 = self.desired_joint1
        if(abs(self.desired_joint2 - data.joint_variable[1]) >= self.err):
            self.joint2 = self.desired_joint2
        if(abs(self.desired_joint3 - data.joint_variable[2]) >= self.err):
            self.joint3 = self.desired_joint3
        if(abs(self.desired_joint4 - data.joint_variable[3]) >= self.err):
            self.joint4 = self.desired_joint4
        if(abs(self.desired_joint5 - data.joint_variable[4]) >= self.err):
            self.joint5 = self.desired_joint5
        if(abs(self.desired_joint6 - data.joint_variable[5]) >= self.err):
            self.joint6 = self.desired_joint6
        
    # Funcao de callback do sensor de forca
    def callback_force(self, data):
        # Variavel que alerta sobre a forca
        self.forceFlag = 0
        # Calculo e analise da forca
        force = sqrt(data.twist.linear.x**2 + data.twist.linear.y**2 + data.twist.linear.z**2)
        if force >= 0.9:
            self.forceFlag = 1
        else:
            self.forceFlag = 0


	



if __name__ == '__main__':

    # Inicializa o no
    rospy.init_node('rosi_ur5_control', anonymous=True)

    # Inicializa o objeto
    try:
        node_obj = RosiNodeClass()
    except rospy.ROSInterruptException:
        pass
