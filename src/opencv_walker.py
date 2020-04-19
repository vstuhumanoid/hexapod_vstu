#!/usr/bin/env python

import rospy
from phantomx_gazebo.phantomx import PhantomX
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

TARGET_POS_TOPIC = '/object'
K = 1

def pose_callback(pose_stamped):
    y = pose_stamped.pose.position.y
    angular = y * K
    robot.set_walk_velocity(0.5, 0, angular)

if __name__ == '__main__':
    rospy.init_node('opencv_walker')

    rospy.loginfo('Initialization')
    robot = PhantomX('/hexapod/')
    pose_sub = rospy.Subscriber(TARGET_POS_TOPIC, PoseStamped, pose_callback)
    rospy.sleep(1)

    rospy.loginfo('Start')
    rospy.spin()

    #robot.set_walk_velocity(0.2, 0, 0)
