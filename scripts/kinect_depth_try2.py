#!/usr/bin/env python
######################################################
# CODIGO QUE MOSTRA A IMAGEM DEPTH VISTA PELO KINECT #
######################################################
from __future__ import print_function # apenas para imprimir os erros, caso hajam
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from random import randint

class image_converter:

  def __init__(self):
      self.bridge = CvBridge()
      # No em que vai publicar a imagem convertida
      self.image_pub = rospy.Publisher("aai_depth_show",Image, queue_size=1)
      # No em que vai subscrever a imagem a ser lida
      self.image_sub = rospy.Subscriber("/sensor/kinect_depth",Image,self.callback)

  # Callback do topico do kinect
  def callback(self,data):
    #try except caso hajam erros
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
        print(e)
    depth_array = np.array(cv_image, dtype=np.float32)
    cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
    cv2.imshow("Kinect Depth", depth_array)
    cv2.waitKey(3)

    prox = depth_array[0][0]
    linha = 0
    coluna = 640
    colunas = list()
    for i in range(int(0.7*data.height)):
        if min(depth_array[i]) < prox:
            prox = min(depth_array[i])
            linha = i

    for i in range(data.width):
        if abs(depth_array[linha][i] - prox) < 0.001:
            colunas.append(i)
            if abs(310 - i) < abs(310 - coluna):
                coluna = i

    print('Valor mais escuro =', prox)
    print('Linha =', linha)
    print('Coluna =', coluna)
    #print(colunas)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('kinect_depth_show', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
