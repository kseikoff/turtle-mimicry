#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
import random

def talker():
    rospy.init_node("targets_node", anonymous=True)
    pub = rospy.Publisher("/targets_topic", Pose, queue_size=1)
    rate = rospy.Rate(0.2)

    target_pose = Pose()
    target_pose.x = [2.0, 4.0, 7.0, 8.0]
    target_pose.y = [3.0, 2.0, 1.0, 4.0]

    target_order = list(zip(target_pose.x, target_pose.y))
    random.shuffle(target_order)

    target_index = 0
    while not rospy.is_shutdown() and target_index < len(target_order):
        random_coord_x, random_coord_y = target_order[target_index]

        pose_msg = Pose()
        pose_msg.x = random_coord_x
        pose_msg.y = random_coord_y

        pub.publish(pose_msg)
        rospy.loginfo("published: {} {}".format(random_coord_x, random_coord_y))

        target_index += 1
        
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
