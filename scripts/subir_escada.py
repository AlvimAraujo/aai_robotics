#!/usr/bin/env python
######################################################################
# CODIGO QUE COMANDA OS BRACOS DAS RODAS DO ROSI                     #
######################################################################
import rospy
from rosi_defy.msg import RosiMovement, RosiMovementArray
from geometry_msgs.msg import Twist, Pose
from math import pi, sqrt

class RosiNodeClass():

    def __init__(self):


        # Comandos que serao enviados para as rodas da direita e da esquerda
        self.omega_front = 0
        self.omega_back = 0
        # Representacao da posicao desejada dos bracos
        self.desired_front = 0
        self.desired_back = 0

        self.state = 0

        # Mensagem de inicializacao
        rospy.loginfo('arms_control iniciado')

        # Publicar em command_arms_speed
        self.pub_arms_vel = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)
        # Subscrever em arms_joints_position
        self.sub_arms_pos = rospy.Subscriber('/rosi/arms_joints_position', RosiMovementArray, self.callback_arms)

        self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)

        # Frequencia de publicacao
        node_sleep_rate = rospy.Rate(10)

        # Loop principal do algoritmo
        while not rospy.is_shutdown():

            # Comando de tracao a ser publicado
            traction_command_list = RosiMovementArray()

            # Criar traction_command_list como uma soma de traction_commands
            for i in range(4):

                # Um comando de tracao por roda
                traction_command = RosiMovement()
                # ID da roda
                traction_command.nodeID = i+1
                # Separa as rodas do lado direito do esquerdo
                if i == 0 or i == 2:
                    traction_command.joint_var = self.omega_front
                else:
                    traction_command.joint_var = self.omega_back
                # Adiciona o comando ao comando de tracao final
                traction_command_list.movement_array.append(traction_command)

            # Publicacao
            self.pub_arms_vel.publish(traction_command_list)

            # Pausa
            node_sleep_rate.sleep()

    # Funcao de callback
    def callback_arms(self, data):

        #print('escada estado = ' + str(self.state))

        # Erro permitido
        self.err = 0.1

        if self.state == 0:
            self.omega_front = 0
            self.omega_back = 0

        if self.state == 1:
            self.desired_front = pi/3
            self.desired_back = pi/4

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = -0.52
            else:
                self.omega_front = 0

            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = 0.52
            else:
                self.omega_back = 0

        if self.state == 2:
            self.desired_front = 0
            self.desired_back = 0.2

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = 0.52
            else:
                self.omega_front = 0

            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = -0.52
            else:
                self.omega_back = 0

        if self.state == 3:
            self.desired_front = -1
            self.desired_back = -pi/2

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = 0.52
            else:
                self.omega_front = 0

            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = -0.52
            else:
                self.omega_back = 0

        if self.state == 4:
            self.desired_front = 0
            self.desired_back = -pi

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = -0.52
            else:
                self.omega_front = 0

            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = 0.52
            else:
                self.omega_back = 0


        if self.state == 5:
            self.desired_front = 0
            self.desired_back = 0

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = 0.52
            else:
                self.omega_front = 0

            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = -0.52
            else:
                self.omega_back = 0


    def callback_pose(self, data):

        Err_pos = 0.15
        self.pos_x  = data.position.x
        self.pos_y = data.position.y

        Pontos = [(0,0), (-38.2, 2), (-41.8, 1.8), (-42.35, 1.8), (-42.75, 1.8), (-43.2, 1.8)]

        (x_goal, y_goal) = Pontos[1]
        if abs(self.pos_x - x_goal) < Err_pos and abs(self.pos_y - y_goal) < Err_pos:
            self.state = 1

        (x_goal, y_goal) = Pontos[2]
        if abs(self.pos_x - x_goal) < Err_pos and abs(self.pos_y - y_goal) < Err_pos:
            self.state = 2

        (x_goal, y_goal) = Pontos[3]
        if abs(self.pos_x - x_goal) < Err_pos and abs(self.pos_y - y_goal) < Err_pos:
            self.state = 3

        (x_goal, y_goal) = Pontos[4]
        if abs(self.pos_x - x_goal) < Err_pos and abs(self.pos_y - y_goal) < Err_pos:
            self.state = 4

        (x_goal, y_goal) = Pontos[5]
        if abs(self.pos_x - x_goal) < Err_pos and abs(self.pos_y - y_goal) < Err_pos:
            self.state = 5




# Funcao principal
if __name__ == '__main__':

    # Inicializa o no
    rospy.init_node('subir_escada', anonymous=True)

    # Inicializa o objeto
    try:
        node_obj = RosiNodeClass()
    except rospy.ROSInterruptException:
        pass