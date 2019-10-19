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
        # Posicao (x, y)
        self.pos_x = 0.1
        self.pos_y = 0.1

        self.fire_list = list()

        #self.sub_pose = rospy.Subscriber('/aai_rosi_pose', Pose, self.callback_pose)
        self.sub_fire_pose = rospy.Subscriber('/aai_fire_pose', Pose, self.callback_fire_pose)

    # def callback_pose(self, data):
    #     self.pos_x  = data.position.x
    #     self.pos_y = data.position.y

    def callback_fire_pose(self, data):
        if (data.position.x, data.position.y) not in self.fire_list:
            self.fire_list.append((data.position.x, data.position.y))
            # print('New fire point' + str(data.position.x) + ' ' + str(data.position.y))

        #mapa = cv2.imread('./src/aai_robotics/images/mapa_rosi.jpg', 1)
        mapa = cv2.imread(rospy.get_param('map_path'), 1)

        scale = 0.5

        mapa = cv2.resize(mapa,(int(scale*1900) ,int(scale*400)))

        w = mapa.shape[1]
        h = mapa.shape[0]

        aux_h = int(10*scale)
        aux_w = int(20*scale)
        aux_r = int(20*scale)

        for (x, y) in self.fire_list:

            Pixel_x = (x + 60.21)* w /65.08
            Pixel_y = 393*h/400 - (y + 6.82)* h /13.08
            Pixel_x = int(Pixel_x)
            Pixel_y = int(Pixel_y)
            cv2.circle(mapa, (Pixel_x,Pixel_y), aux_r , (0,0,255), 2)

        # # Pose do robo quando viu o fogo
        # Pixel_x = (self.pos_x + 60.21)* w /65.08
        # Pixel_y = 393*h/400 - (self.pos_y + 6.82)* h /13.08
        # Pixel_x = int(Pixel_x)
        # Pixel_y = int(Pixel_y)

        cv2.imshow("Mapa", mapa)
        cv2.waitKey(0)

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
