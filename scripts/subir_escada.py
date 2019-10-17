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

        print('escada estado = ' + str(self.state))

        # Erro permitido
        self.err = 0.1

        if self.state == 0:
            self.omega_front = 0
            self.omega_back = 0

        if self.state == 1:
            self.desired_front = pi/2
            self.desired_back = pi/2

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = -0.52
            else:
                self.omega_front = 0
                #print('certim na frente')
            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = 0.52
            else:
                self.omega_back = 0
                #print('certim atras')

        if self.state == 2 or self.state == 2.1:
            self.desired_front = 0
            self.desired_back = 0.16

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = 0.40
            else:
                self.omega_front = 0
                #print('certim na frente')
            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = -0.50
            else:
                self.omega_back = 0
                self.state = 3
                #print('certim atras')

        if self.state == 3 or self.state == 3.1:
            self.desired_front = -pi/6
            self.desired_back = pi/6

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = 0.30
            else:
                self.omega_front = 0
                #print('certim na frente')
            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = 0.30
            else:
                self.omega_back = 0
                self.state = 4
                #print('certim atras')

        if self.state == 4:
            self.desired_front = -pi/8
            self.desired_back = pi/8

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = 0.30
            else:
                self.omega_front = 0
                #print('certim na frente')
            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = -0.30
            else:
                self.omega_back = 0
                self.state = 5

        if self.state == 5:
            self.desired_front = -pi/4
            self.desired_back = 0

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = 0.52
            else:
                self.omega_front = 0
                #print('certim na frente')
            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = -0.52
            else:
                self.omega_back = 0
                self.state = 6

        if self.state == 6:
            #self.desired_front = -pi/8
            self.desired_back = -pi/4

            #if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
            #    self.omega_front = -0.52
            #else:
                #self.omega_front = 0
                #print('certim na frente')
            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = -0.52
                self.omega_front = 0
            else:
                self.omega_back = 0
                self.state = 7

        if self.state == 7:
            self.desired_back = -pi/6

            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = 0.30
                self.omega_front = 0
            else:
                self.omega_back = 0
                self.state = 8


        if self.state == 8:
            self.desired_back = 0

            if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
                self.omega_back = 0.10
                self.omega_front = 0
            else:
                self.omega_back = 0
                self.state = 9

        if self.state == 9:
            self.desired_front = 0

            if abs(data.movement_array[0].joint_var - self.desired_front) >= self.err:
                self.omega_front = -0.52
                self.omega_back = 0
            else:
                self.omega_front = 0
                #print('certim na frente')
                self.state = 10

    def callback_pose(self, data):

        Err_pos = 0.2
        self.pos_x  = data.position.x
        self.pos_y = data.position.y

        Pontos = [(0,0), (-38.2, 2), (-41.5, 2)]

        (x_goal, y_goal) = Pontos[1]
        if abs(self.pos_x - x_goal) < Err_pos and abs(self.pos_y - y_goal) < Err_pos:
            self.state = 1

        (x_goal, y_goal) = Pontos[2]
        if abs(self.pos_x - x_goal) < Err_pos and abs(self.pos_y - y_goal) < Err_pos:
            self.state = 2




# Funcao principal
if __name__ == '__main__':

    # Inicializa o no
    rospy.init_node('subir_escada', anonymous=True)

    # Inicializa o objeto
    try:
        node_obj = RosiNodeClass()
    except rospy.ROSInterruptException:
        pass
