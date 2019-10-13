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

        self.image = list()#[10000, 10000]

        # Nos em que vai se subscrever e publicar
        self.sub_kinect = rospy.Subscriber('/sensor/kinect_depth', Image, self.callback_kinect)

        # Frequencia de publicacao
        node_sleep_rate = rospy.Rate(1)

        # Mensagem de inicializacao
        rospy.loginfo('get_pose_from_sensors iniciado')

        # Loop principal, responsavel pelos procedimentos chaves do programa
        while not rospy.is_shutdown():
            try:
                i = self.image.index(min(self.image))
                rospy.loginfo('Indice ' + str(i) + ' ' + 'Com valor ' + str(self.image[i]))
                rospy.loginfo('Pixel mais preto => ' + str(i%480) + ' ' + str(i//640))
            except:
                pass
            node_sleep_rate.sleep()


    # Callback do sensor kinect_depth
    def callback_kinect(self, data):
        print(data.data[0])
        aux1 = int(data.data[0:491520:2])
        aux2 = int(data.data[1:491520:2])
        self.image = list()
        for i in range(len(aux1)):
            self.image.append(aux1[i] + 256*aux2[i])

# Funcao main
if __name__ == '__main__':

    # Inicializacao do no
    rospy.init_node('get_aai_pose', anonymous=True)

    # Objeto
    try:
        node_obj = RosiPoseClass()
    except rospy.ROSInterruptException:
        pass
