#!/usr/bin/env python
#####################################################################
# CODIGO QUE UNE AS INFORMACOES UTEIS DE POSICAO EM UM UNICO TOPICO #
#####################################################################
import rospy
from sensor_msgs.msg import Image
# from geometry_msgs.msg import Pose

class RosiPoseClass():

	# Class constructor
    def __init__(self):
        # Parametros necessarios de posicao e orientacao
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.orientation_quaternion_x = 0.0
        self.orientation_quaternion_y = 0.0
        self.orientation_quaternion_z = 0.0
        self.orientation_quaternion_w = 0.0
        self.vetor = list()

        # Nos em que vai se subscrever e publicar
        self.sub_gps = rospy.Subscriber('/sensor/kinect_depth', Image, self.callback_kinect)

        # Frequencia de publicacao
        node_sleep_rate = rospy.Rate(10)

        # Mensagem de inicializacao
        rospy.loginfo('get_pose_from_sensors iniciado')



        # Loop principal, responsavel pelos procedimentos chaves do programa
        while not rospy.is_shutdown():
            pass
            # actual_pose = Pose()
            #
            # # Posicao cartesiana
            # actual_pose.position.x = self.position_x
            # actual_pose.position.y = self.position_y
            # actual_pose.position.z = self.position_z
            #
            # # Orientacao em quaternios
            # actual_pose.orientation.x = self.orientation_quaternion_x
            # actual_pose.orientation.y = self.orientation_quaternion_y
            # actual_pose.orientation.z = self.orientation_quaternion_z
            # actual_pose.orientation.w = self.orientation_quaternion_w

            # #Publicacao
            # self.pub_pose.publish(actual_pose)

    # Callback do sensor gps
    def callback_kinect(self, data):

        C = open('text/data.txt', 'w')
        C.write(str(data.data))
        C.close()
        print('Escritos')

# Funcao main
if __name__ == '__main__':

    # Inicializacao do no
    rospy.init_node('get_aai_pose', anonymous=True)

    # Objeto
    try:
        node_obj = RosiPoseClass()
    except rospy.ROSInterruptException:
        pass
