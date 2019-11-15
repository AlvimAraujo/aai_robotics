#!/usr/bin/env python
######################################################
# CODIGO QUE MOSTRA A IMAGEM DEPTH VISTA PELO KINECT #
######################################################
# No que pega a imagem de profundidade dada pelo kinect
# e a converte para 32FC1

from __future__ import print_function # apenas para imprimir os erros, caso existam
# Pacotes
import roslib
import sys
import rospy
import cv2
import numpy as np
# Tipos de mensagens utilizadas
from std_msgs.msg import String
from sensor_msgs.msg import Image
# Ferramentas para o processamento dos dados
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
      self.bridge = CvBridge()
      # Topico em que vai publicar a imagem convertida
      self.image_pub = rospy.Publisher("/aai_depth_show",Image, queue_size=1)
      # Topico em que vai subscrever a imagem a ser lida
      self.image_sub = rospy.Subscriber("/sensor/kinect_depth",Image,self.callback)

  # Callback do topico do kinect
  def callback(self,data):
    # try except caso hajam erros
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

        depth_array = np.array(cv_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        # Visualizacao da imagem
        if rospy.get_param('mostar_kinect_depth'):
            cv2.imshow("Kinect Depth", depth_array)
            cv2.waitKey(3)
        else:
            cv2.destroyAllWindows()

    except CvBridgeError as e:
        print(e)
    try:
        # Imagem convertida
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="32FC1"))
    except CvBridgeError as e:
        print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('kinect_depth_show', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
