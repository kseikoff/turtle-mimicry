#!/usr/bin/env python3

import random
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# coords (2, 3), (4, 2), (7, 1), (8, 4)

class OriginTurtle(object):
    def __init__(self, node_name):
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(30)

        self.cmd_vel_pub = rospy.Publisher("/{}/cmd_vel".format(node_name), Twist, queue_size=1)
        self.publisher_target = rospy.Publisher("/targets_topic", Pose, queue_size=1)

        self.target_pose = Pose()
        self.target_pose.x = [2.0, 4.0, 7.0, 8.0]
        self.target_pose.y = [3.0, 2.0, 1.0, 4.0]

        self.target_order = list(zip(self.target_pose.x, self.target_pose.y))
        random.shuffle(self.target_order)

        self.pose = Pose()
        self.pose.x = 0
        self.pose.y = 0
        self.target_index = 0
    
    def spin(self):
        while not rospy.is_shutdown() and self.target_index < len(self.target_order) - 1:
            twist_msg = Twist()
            pose_msg = Pose()

            random_coord_x, random_coord_y = self.target_order[self.target_index]

            twist_msg.linear.x = random_coord_x - self.pose.x
            self.pose.x = random_coord_x
            pose_msg.x = random_coord_x

            twist_msg.linear.y = random_coord_y - self.pose.y
            self.pose.y = random_coord_y
            pose_msg.y = random_coord_y

            self.target_index += 1

            self.cmd_vel_pub.publish(twist_msg)
            self.publisher_target.publish(pose_msg)

            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


class ImitatorTurtle(object):
    def __init__(self, node_name):
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(30)
        
        self.cmd_vel_pub = rospy.Publisher("/{}/cmd_vel".format(node_name), Twist, queue_size=1)
        self.subscriber = rospy.Subscriber("/{}/pose".format(node_name), Pose, self.pose_callback)
        self.subscriber_target = rospy.Subscriber("/targets_topic", Pose, self.target_callback)

        self.target_pose = Pose()
        self.target_pose.x = 0
        self.target_pose.y = 0

        self.pose = Pose()
        self.pose.x = 0
        self.pose.y = 0
    
    def target_callback(self, msg:Pose):
        rospy.loginfo("target updated with: {0} {1}".format(msg.x, msg.y))
        self.update_target_control(msg.x, msg.y)
    
    def pose_callback(self, msg:Pose):
        rospy.loginfo("curr pose: {0} {1}".format(msg.x, msg.y))
        self.update_control(msg.x, msg.y)
    
    def update_target_control(self, x, y):
        self.target_pose.x, self.target_pose.y = x, y

    def update_control(self, x, y):
        self.pose.x, self.pose.y = x, y
    
    def spin(self):
        while not rospy.is_shutdown():
            twist_msg = Twist()

            twist_msg.linear.x = self.target_pose.x - self.pose.x
            twist_msg.linear.y = self.target_pose.y - self.pose.y

            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("turtles", anonymous=True)
    origin_turtle = OriginTurtle("origin_turtle")
    imitator_turtle = ImitatorTurtle("imitator_turtle")

    origin_turtle.spin()
    imitator_turtle.spin()