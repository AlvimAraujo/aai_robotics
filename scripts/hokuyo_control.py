#!/usr/bin/env python
##################################################################
# CONTROLA A CAMERA PARA SEMPRE FILMAR A ESTEIRA, LENDO O HOKUYO #
##################################################################
import rospy
from rosi_defy.msg import HokuyoReading, ManipulatorJoints
from math import pi, sin, cos, sqrt, atan2
from geometry_msgs.msg import TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class RosiPoseClass():

	# Class constructor
    def __init__(self):
        self.near_x = 5
        self.near_y = 0
        self.near_z = 0

        self.angle = 0

        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.joint5 = 0
        self.joint6 = 0

        # Nos em que vai se subscrever e publicar
        self.sub_hokuyo = rospy.Subscriber('/sensor/hokuyo', HokuyoReading, self.callback_hokuyo)

        # Publica em jointsPosTargetCommand
        self.pub_jointsPos = rospy.Publisher('/ur5/jointsPosTargetCommand',  ManipulatorJoints, queue_size=1)

        # Subscreve em jointsPositionCurrentState
        self.sub_joints = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_joints)

        # Frequencia de publicacao
        node_sleep_rate = rospy.Rate(5)

        # Mensagem de inicializacao
        rospy.loginfo('camera control iniciado')

        # Loop principal, responsavel pelos procedimentos chaves do programa
        while not rospy.is_shutdown():
            traj = ManipulatorJoints()
            traj.header.stamp = rospy.Time.now()
            traj.joint_variable = [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6]

            # Publica a mensagem
            self.pub_jointsPos.publish(traj)

            # Pausa
            node_sleep_rate.sleep()
            #print(self.near_x, self.near_y, self.near_z)

            if not rospy.get_param('touch_mode'):
                break

    # Funcao de callback da posicao das juntas
    def callback_joints(self, data):
        # Erro permitido
        self.err = 0.05
        
        # Posicao desejada das juntas
        self.desired_joint1 = self.angle
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

    # Callback do sensor hokuyo
    def callback_hokuyo(self, data):
        borda_x = data.reading[0::3]
        borda_y = data.reading[1::3]
        borda_z = data.reading[2::3]

        near_x = borda_x[0]
        near_y = borda_y[0]
        near_z = borda_z[0]

        for i in range(len(borda_x)):
            d_x = borda_x[i]
            d_y = borda_y[i]
            d_z = borda_z[i]

            if sqrt(d_x**2 + d_y**2) - sqrt(near_x**2 + near_y**2) < 0.1:
                near_x = d_x
                near_y = d_y
                near_z = d_z

        self.near_x = near_x
        self.near_y = near_y
        self.near_z = near_z

        self.angle = atan2(near_y, near_x)

# Funcao main
if __name__ == '__main__':

    # Inicializacao do no
    rospy.init_node('hokuyo_control', anonymous=True)

    # Objeto
    try:
        node_obj = RosiPoseClass()
    except rospy.ROSInterruptException:
        pass
