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
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class image_converter:
    def __init__(self):
        # Posicao (x, y, theta)
        self.pos_x = 0.1
        self.pos_y = 0.1
        self.angle = 0.1

        self.bridge = CvBridge()
        # No em que vai subscrever a imagem a ser lida
        self.image_sub = rospy.Subscriber('/sensor/ur5toolCam',Image,self.callback_rgb)
        self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)

        self.pub_fire_pose = rospy.Publisher('/aai_fire_pose', Pose, queue_size=1)
            #while not rospy.is_shutdown():

    def callback_pose(self, data):

		q_x = data.orientation.x
		q_y = data.orientation.y
		q_z = data.orientation.z
		q_w = data.orientation.w
		# Orientacao de quaternios para angulos de Euler
		euler_angles = euler_from_quaternion([q_x, q_y, q_z, q_w])

		self.pos_x  = data.position.x
		self.pos_y = data.position.y
		self.angle = euler_angles[2] # Apenas o angulo de Euler no eixo z interessa

    # Callback do topico do kinect
    def callback_rgb(self,data):
        #try except caso hajam erros
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv_image[0:400, 0:640]
        except CvBridgeError as e:
            print(e)
        # # Desenhar circulo
        # cv2.circle(cv_image, (50,50), 10, 255)
        blur = cv2.GaussianBlur(cv_image, (21, 21), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lower = np.array([18, 50, 50], dtype="uint8")
        upper = np.array([35, 255, 255], dtype="uint8")
        mask = cv2.inRange(hsv, lower, upper)

        output = cv2.bitwise_and(cv_image, hsv, mask=mask)
        no_red = cv2.countNonZero(mask)
        #cv2.imshow("ur5 cam", cv_image)
        cv2.imshow("Fire", output)
        cv2.waitKey(3)
        if int(no_red) > 2000:
            print('*'*10)
            print('Alerta! Possivel rolo em chamas detectado')
            print('Proximo a (latitude, longitude) = ' + str(self.pos_x) + ' ' + str(self.pos_y))
            print('Regiao marcada no mapa')
            print('*'*10)
            FP = Pose()
            FP.position.x = self.pos_x
            FP.position.y = self.pos_y
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
