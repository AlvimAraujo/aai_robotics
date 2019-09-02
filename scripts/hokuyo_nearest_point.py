#!/usr/bin/env python
#####################################################################
# CODIGO QUE UNE AS INFORMACOES UTEIS DE POSICAO EM UM UNICO TOPICO #
#####################################################################
import rospy
from rosi_defy.msg import HokuyoReading
from math import pi
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose

class RosiPoseClass():

	# Class constructor
    def __init__(self):
        self.data = HokuyoReading()
        # Parametros necessarios de posicao e orientacao
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.orientation_quaternion_x = 0.0
        self.orientation_quaternion_y = 0.0
        self.orientation_quaternion_z = 0.0
        self.orientation_quaternion_w = 0.0

        # Nos em que vai se subscrever e publicar
        self.pub_list = rospy.Publisher('/aai_rosi_border_obstacle', HokuyoReading, queue_size=1)
        self.sub_hokuyo = rospy.Subscriber('/sensor/hokuyo', HokuyoReading, self.callback_hokuyo)

        # Frequencia de publicacao
        node_sleep_rate = rospy.Rate(10)

        # Mensagem de inicializacao
        rospy.loginfo('get_pose_from_sensors iniciado')

        # Loop principal, responsavel pelos procedimentos chaves do programa
        while not rospy.is_shutdown():
            #Publicacao
            self.pub_list.publish(self.data)

    # Callback do sensor gps
    def callback_hokuyo(self, data):
        borda_x = data.data[0::3]
        borda_y = data.data[1::3]
        borda_z = data.data[2::3]


        x_world = list()
        y_world = list()
        z_world = list()

        x_world += [cos(T)*borda_x - sin(T)*borda_y + d_x]
        y_world += [sin(T)*borda_x + cos(T)*borda_y + d_y]
        z_world += [borda_z + d_z]

        #p0 = R01p1 + d01
        #Rz = [c -s 0; s c 0; 0 0 1]

        self.data.data = data.data



# Funcao main
if __name__ == '__main__':

    # Inicializacao do no
    rospy.init_node('get_aai_pose', anonymous=True)

    # Objeto
    try:
        node_obj = RosiPoseClass()
    except rospy.ROSInterruptException:
        pass
