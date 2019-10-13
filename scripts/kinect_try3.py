#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("aai_depth_show",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/sensor/kinect_depth",Image,self.callback)

  def callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="mono16")
    except CvBridgeError as e:
        print(e)
    # try:
    #     cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    #     cv_image_array = np.array(cv_image, dtype = np.dtype('uint8'))
    #     cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    #     cv_image_resized = cv2.resize(cv_image_norm, self.desired_shape, interpolation = cv2.INTER_CUBIC)
    #     self.depthimg = cv_image_resized
    #     cv2.imshow("Image from my node", self.depthimg)
    #     cv2.waitKey(1)
    # except CvBridgeError as e:
    #     print(e)

    cv2.imshow("Kinect Depth", cv_image)
    cv2.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
