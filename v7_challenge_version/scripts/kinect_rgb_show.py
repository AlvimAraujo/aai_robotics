#!/usr/bin/env python
####################################################
# CODIGO QUE MOSTRA A IMAGEM RBG VISTA PELO KINECT #
####################################################
from __future__ import print_function # apenas para imprimir os erros, caso existam
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
      self.bridge = CvBridge()
      # No em que vai publicar a imagem convertida
      self.image_pub = rospy.Publisher("aai_image_show",Image, queue_size=1)
      # No em que vai subscrever a imagem a ser lida
      self.image_sub = rospy.Subscriber("/sensor/kinect_rgb",Image,self.callback_rgb)

  # Callback do topico do kinect
  def callback_rgb(self,data):
    #try except caso hajam erros
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Desenhar circulo
    # cv2.circle(cv_image, (50,50), 10, 255)
    cv2.imshow("Kinect RGB", cv_image)
    cv2.waitKey(3)

    #try except caso hajam erros
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('kinect_rgb_show', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
