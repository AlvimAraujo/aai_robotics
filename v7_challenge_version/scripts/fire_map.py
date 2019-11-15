#!/usr/bin/env python
#####################################################
# CODIGO QUE GERA O MAPA COM OS PONTOS ONDE HA FOGO #
#####################################################

from __future__ import print_function # apenas para imprimir os erros, caso existam
# Pacotes
import roslib
import sys
import rospy
import cv2
import numpy as np
# Tipos de mensagens utilizadas
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
# Ferramentas para o processamento dos dados
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class image_converter:
    # Construtor
    def __init__(self):
        # Posicao (x, y)
        self.pos_x = 0.1
        self.pos_y = 0.1
        # Pontos dos possiveis rolos em chamas
        self.fire_list = list()
        # Topico em que contem as coordenadas dos possiveis rolos em chamas
        self.sub_fire_pose = rospy.Subscriber('/aai_fire_pose', Pose, self.callback_fire_pose)

    # Callback do topico
    def callback_fire_pose(self, data):
        if (data.position.x, data.position.y) not in self.fire_list:
            self.fire_list.append((data.position.x, data.position.y))
            #print('New fire point' + str(data.position.x) + ' ' + str(data.position.y))

        #mapa = cv2.imread('./src/aai_robotics/images/mapa_rosi.jpg', 1) # Caminho relativo
        # Leitura do arquivo contendo o mapa
        mapa = cv2.imread(rospy.get_param('map_path'), 1)
        # Fator de escala
        scale = rospy.get_param('map_scale_factor')
        # Tamanho da imagem
        mapa = cv2.resize(mapa,(int(scale*1900) ,int(scale*446)))
        w = mapa.shape[1]
        h = mapa.shape[0]
        # aux_h = int(10*scale)
        # aux_w = int(20*scale)
        aux_r = int(22*scale)

        for (x, y) in self.fire_list:
            Pixel_x = (x + 60.21)* w /65.08
            Pixel_y = 393*h/400 - (y + 6.82)* h /13.08
            Pixel_x = int(Pixel_x)
            Pixel_y = int(Pixel_y)
            cv2.circle(mapa, (Pixel_x,Pixel_y), aux_r , (255,0,0), 2)

        # Pose do robo quando viu o fogo
        # Pixel_x = (self.pos_x + 60.21)* w /65.08
        # Pixel_y = 393*h/446 - (self.pos_y + 6.82)* h /13.08
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
