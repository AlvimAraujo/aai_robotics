#!/usr/bin/env python
#####################################################################
# CODIGO QUE UNE AS INFORMACOES UTEIS DE POSICAO EM UM UNICO TOPICO #
#####################################################################
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose

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

        # Nos em que vai se subscrever e publicar
        self.pub_pose = rospy.Publisher('/aai_rosi_pose', Pose, queue_size=10)
        self.sub_gps = rospy.Subscriber('/sensor/gps', NavSatFix, self.callback_gps)
        self.sub_imu = rospy.Subscriber('/sensor/imu', Imu, self.callback_imu)

        # Frequencia de publicacao
        node_sleep_rate = rospy.Rate(10)

        # Mensagem de inicializacao
        rospy.loginfo('get_pose_from_sensors iniciado')

        # Loop principal, responsavel pelos procedimentos chaves do programa
        while not rospy.is_shutdown():
            actual_pose = Pose()

            # Posicao cartesiana
            actual_pose.position.x = self.position_x
            actual_pose.position.y = self.position_y
            actual_pose.position.z = self.position_z

            # Orientacao em quaternios
            actual_pose.orientation.x = self.orientation_quaternion_x
            actual_pose.orientation.y = self.orientation_quaternion_y
            actual_pose.orientation.z = self.orientation_quaternion_z
            actual_pose.orientation.w = self.orientation_quaternion_w

            #Publicacao
            self.pub_pose.publish(actual_pose)

    # Callback do sensor gps
    def callback_gps(self, data):
        self.position_x = data.latitude
        self.position_y = data.longitude
        self.position_z = data.altitude

    # Callback do sensor imu
    def callback_imu(self, data):
        self.orientation_quaternion_x = data.orientation.x
        self.orientation_quaternion_y = data.orientation.y
        self.orientation_quaternion_z = data.orientation.z
        self.orientation_quaternion_w = data.orientation.w

# Funcao main
if __name__ == '__main__':

    # Inicializacao do no
    rospy.init_node('get_aai_pose', anonymous=True)

    # Objeto
    try:
        node_obj = RosiPoseClass()
    except rospy.ROSInterruptException:
        pass
