#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class RosiNodeClass():

    # Atributos segundo o manual
	max_translational_speed = 5 # in [m/s]
	max_rotational_speed = 10 # in [rad/s]
	var_lambda = 0.965
	wheel_radius = 0.1324
	ycir = 0.531

	# Class constructor
	def __init__(self):

        self.latitude = 0.0
        self.longitude = 0.0
        self.orientation = 0.0

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        sel.sub_gps = rospy.Subscriber('/sensor/gps', NavSatFix, self.callback_gps)

        node_sleep_rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            vel_msg = Twist()
            self.pub_cmd_vel.publish(vel_msg)

	def callback_gps(self, data):

        self.latitude = data.latitude
        self.longitude = data.longitude

if __name__ == '__main__':

    rospy.init_node('calculo_vel', anonymous=True)

    try:
        node_obj = RosiNodeClass()
    except rospy.ROSInterruptException:
        pass
