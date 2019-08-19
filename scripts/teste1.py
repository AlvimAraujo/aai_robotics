#!/usr/bin/env python
import rospy
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray

def talker():
    rospy.init_node('teste1_mover', anonymous=True)
    pub = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
    rate = rospy.Rate(20)
    traction_command_list = RosiMovementArray()
    while not rospy.is_shutdown():
        for i in range(4):
            traction_command = RosiMovement()
            traction_command.nodeID = i+1
            if i < 2: #RODAS DA DIREITA
                traction_command.joint_var = 30.0
            else: #RODAS DA ESQUERDA
                traction_command.joint_var = 30.0
            traction_command_list.movement_array.append(traction_command)
        pub.publish(traction_command_list)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
