#!/usr/bin/env python
######################################################################
# CODIGO QUE COMANDA OS BRACOS DAS RODAS DO ROSI                     #
######################################################################
import rospy
from rosi_defy.msg import RosiMovement, RosiMovementArray
from geometry_msgs.msg import Twist
from math import pi

class RosiNodeClass():

    def __init__(self):

        # Comandos que serao enviados para as rodas da direita e da esquerda
        self.omega_front = 0
        self.omega_back = 0
        # Representacao da posicao desejada dos bracos
        self.desired_front = 0
        self.desired_back = 0

        # Mensagem de inicializacao
        rospy.loginfo('arms_control iniciado')

        # Publicar em command_arms_speed
        self.pub_arms_vel = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)
        # Subscrever em arms_joints_position
        self.sub_arms_pos = rospy.Subscriber('/rosi/arms_joints_position', RosiMovementArray, self.callback_arms)

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

        # Erro permitido
        self.err = 0.05

        # Rotina de subida da escada
        self.desired_front = 0 #-pi/2
        self.desired_back = 0 #pi/2

        if abs(data.movement_array[2].joint_var - self.desired_front) >= self.err:
            self.omega_front = -0.52
        else:
            self.omega_front = 0
            #print('certim na frente')
        if abs(data.movement_array[3].joint_var - self.desired_back) >= self.err:
            self.omega_back = -0.52
        else:
            self.omega_back = 0
            #print('certim atras')

        #print(self.omega_front)
        #print(self.omega_back)
        #print(data.movement_array[1].joint_var)
        #print(abs(data.movement_array[0].joint_var - pi/2)


# Funcao principal
if __name__ == '__main__':

    # Inicializa o no
    rospy.init_node('rosi_arms_control_node', anonymous=True)

    # Inicializa o objeto
    try:
        node_obj = RosiNodeClass()
    except rospy.ROSInterruptException:
        pass
