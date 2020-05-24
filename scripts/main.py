#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

j_leg = ['j_c1', 'j_thigh', 'j_tibia']
suffix = ['f', 'm', 'r']
sides = ['l', 'r']

joints = []

for j in j_leg:
    for s in suffix:
        for side in sides:
            joints.append(j + "_" + side + s)

ns = '/hexapod/'

def talker():
    pub_joints = {}

    for j in joints:
        topic = ns + j + '_position_controller/command'
        p = rospy.Publisher(
            ns + j + '_position_controller/command', Float64, queue_size=1)
        pub_joints[j] = p

    rospy.init_node('Talker', anonymous=True)
    rate = rospy.Rate(10)
    
    f = open('DRIVEMAG.TXT','r')
    data = f.readline()
    command = data.split()
    print(command[3])
    f.close()

    while not rospy.is_shutdown():
        position = 0.0
        for j in joints:
            pub_joints[j].publish(position)
        rospy.loginfo('position sended')
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
