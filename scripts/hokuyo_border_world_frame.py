#!/usr/bin/env python
#####################################################################
# CODIGO QUE UNE AS INFORMACOES UTEIS DE POSICAO EM UM UNICO TOPICO #
#####################################################################
import rospy
from rosi_defy.msg import HokuyoReading
from math import pi, sin, cos
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class RosiPoseClass():

	# Class constructor
    def __init__(self):
        self.data = list()
        self.orientation = 0.0
        self.pos_x = 0.1
        self.pos_y = 0.1
        self.pos_z = 0.1

        # Nos em que vai se subscrever e publicar
        self.pub_hok = rospy.Publisher('/aai_rosi_border_obstacle', HokuyoReading, queue_size=1)
        self.sub_hokuyo = rospy.Subscriber('/sensor/hokuyo', HokuyoReading, self.callback_hokuyo)
        self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)

        # Frequencia de publicacao
        node_sleep_rate = rospy.Rate(10)

        # Mensagem de inicializacao
        rospy.loginfo('hokuyo world frame iniciado')

        # Loop principal, responsavel pelos procedimentos chaves do programa
        while not rospy.is_shutdown():
            #Publicacao
            msg = HokuyoReading()
            msg.reading = self.data
            self.pub_hok.publish(msg)

    # Callback do sensor gps
    def callback_hokuyo(self, data):
        borda_x = data.reading[0::3]
        borda_y = data.reading[1::3]
        borda_z = data.reading[2::3]


        # x_world = list()
        # y_world = list()
        # z_world = list()

        d_x = self.pos_x
        d_y = self.pos_y
        d_z = self.pos_z

        self.data = list()

        for i in range(len(borda_x)):
            x_world = [cos(self.angle + pi/2)*borda_x[i] - sin(self.angle + pi/2)*borda_y[i] + d_x]
            y_world = [sin(self.angle + pi/2)*borda_x[i] + cos(self.angle + pi/2)*borda_y[i] + d_y]
            z_world = [borda_z[i] + d_z]
            # self.data += [x_world, y_world, z_world]
            self.data += x_world
            self.data += y_world
            self.data += z_world

        #p0 = R01p1 + d01
        #Rz = [c -s 0; s c 0; 0 0 1]

        # self.data.reading = data.reading

    def callback_pose(self, data):

        q_x = data.orientation.x
        q_y = data.orientation.y
        q_z = data.orientation.z
        q_w = data.orientation.w
		# Orientacao de quaternios para angulos de Euler
        euler_angles = euler_from_quaternion([q_x, q_y, q_z, q_w])

        self.angle = euler_angles[2]

        self.pos_x  = data.position.x
        self.pos_y = data.position.y
        self.pos_z = data.position.z



# Funcao main
if __name__ == '__main__':

    # Inicializacao do no
    rospy.init_node('border_obstacle', anonymous=True)

    # Objeto
    try:
        node_obj = RosiPoseClass()
    except rospy.ROSInterruptException:
        pass
