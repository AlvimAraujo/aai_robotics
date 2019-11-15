#!/usr/bin/python
#############################
# REALIZA O CONTROLE DO UR5 #
#############################

import rospy
from rosi_defy.msg import ManipulatorJoints
from geometry_msgs.msg import TwistStamped
from math import pi

class RosiNodeClass():

    def __init__(self):
        # Comandos a serem enviados para as juntas
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.joint5 = 0
        self.joint6 = 0

        # Publica em jointsPosTargetCommand
        self.pub_jointsPos = rospy.Publisher('/ur5/jointsPosTargetCommand',  ManipulatorJoints, queue_size=1)

        # Subscreve em jointsPositionCurrentState
        self.sub_joints = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_joints)

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

    # Funcao de callback
    def callback_joints(self, data):
        # Erro permitido
        self.err = 0.05
        # Posicao desejada das juntas
        self.desired_joint1 = 0
        self.desired_joint2 = 0
        self.desired_joint3 = 0
        self.desired_joint4 = 0
        self.desired_joint5 = 0
        self.desired_joint6 = 0

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


if __name__ == '__main__':

    # Inicializa o no
    rospy.init_node('rosi_ur5_control', anonymous=True)

    # Inicializa o objeto
    try:
        node_obj = RosiNodeClass()
    except rospy.ROSInterruptException:
        pass
