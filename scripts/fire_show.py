#!/usr/bin/env python
####################################################
# CODIGO QUE MOSTRA A IMAGEM RBG VISTA PELO KINECT #
####################################################
from __future__ import print_function # apenas para imprimir os erros, caso hajam
import roslib
import sys
import rospy
import cv2
import numpy as np
from rosi_defy.msg import HokuyoReading, ManipulatorJoints
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sqrt, sin, cos, pi, atan2

class image_converter:
    def __init__(self):
        # Posicao (x, y, theta)
        self.pos_x = 0.1
        self.pos_y = 0.1
        self.angle = 0.1

        self.joint = 0


        self.near_x = 10
        self.near_y = 0

        self.bridge = CvBridge()
        # No em que vai subscrever a imagem a ser lida
        self.pub_fire_pose = rospy.Publisher('/aai_fire_pose', Pose, queue_size=1)
        self.image_sub = rospy.Subscriber('/sensor/ur5toolCam',Image,self.callback_rgb)
        self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
        self.sub_hokuyo = rospy.Subscriber('/sensor/hokuyo', HokuyoReading, self.callback_hokuyo)

        self.sub_joints = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_joints)

    def callback_joints(self, data):
        self.joint = data.joint_variable[0]

    def callback_pose(self, data):

		q_x = data.orientation.x
		q_y = data.orientation.y
		q_z = data.orientation.z
		q_w = data.orientation.w
		# Orientacao de quaternios para angulos de Euler
		euler_angles = euler_from_quaternion([q_x, q_y, q_z, q_w])

		self.pos_x  = data.position.x
		self.pos_y = data.position.y
		self.angle = euler_angles[2] + pi/2 + self.joint#+ atan2(self.near_y, self.near_x)# Apenas o angulo de Euler no eixo z interessa

    def callback_hokuyo(self, data):
        borda_x = data.reading[0::3]
        borda_y = data.reading[1::3]

        near_x = borda_x[0]
        near_y = borda_y[0]

        for i in range(len(borda_x)):
            d_x = borda_x[i]
            d_y = borda_y[i]

            if sqrt(d_x**2 + d_y**2) - sqrt(near_x**2 + near_y**2) < 0.01:
                near_x = d_x
                near_y = d_y

        self.near_x = near_x
        self.near_y = near_y

    # Callback do topico do kinect
    def callback_rgb(self,data):
        #try except caso hajam erros
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv_image[0:400, 0:640]
        except CvBridgeError as e:
            print(e)
        blur = cv2.GaussianBlur(cv_image, (21, 21), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lower = np.array([18, 50, 50], dtype="uint8")
        upper = np.array([35, 255, 255], dtype="uint8")
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(cv_image, hsv, mask=mask)
        no_red = cv2.countNonZero(mask)

        center_cut = 100

        blur_center = cv2.GaussianBlur(cv_image[0:400, center_cut:640-center_cut], (21, 21), 0)
        hsv_center = cv2.cvtColor(blur_center, cv2.COLOR_BGR2HSV)
        mask_center = cv2.inRange(hsv_center, lower, upper)
        no_red_center = cv2.countNonZero(mask_center)
        #cv2.imshow("ur5 cam", cv_image)
        if rospy.get_param('mostar_fire_cam'):
            cv2.imshow("Fire", output)
            cv2.waitKey(3)
        else:
            cv2.destroyAllWindows()

        if int(no_red) > 2000:
            print('*'*3)
            print('Alerta! Possivel rolo em chamas detectado')
            print('Proximo a (latitude, longitude) = ' + str(self.pos_x) + ' ' + str(self.pos_y))
            #print('Regiao marcada no mapa')
            print('*'*3)

            if int(no_red_center) > 500:
                xd = cos(self.angle)*self.near_x - sin(self.angle)*self.near_y
                yd = sin(self.angle)*self.near_x + cos(self.angle)*self.near_y
                FP = Pose()
                FP.position.x = self.pos_x + xd
                FP.position.y = self.pos_y + yd

                print('-'*10)
                print('Possivel rolo em chamas indentificado')
                print('Coordenadas  = ' + str(FP.position.x) + ' ' + str(FP.position.y))
                print('')
                #print('Regiao marcada no mapa')
                print('-'*10)

                self.pub_fire_pose.publish(FP)



def main(args):
  ic = image_converter()
  rospy.init_node('fire_detect', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
