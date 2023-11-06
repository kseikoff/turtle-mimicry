#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class SimpleController():
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        curr_ns = rospy.get_namespace()
        self.create_in_namespace(curr_ns)
        self.rate = rospy.Rate(30)

        self.init_coords()

    def create_in_namespace(self, ns:str):
        ns1 = "ns1_368731"
        ns2 = "ns2_368731"

        if ns1 in ns:
            self.cmd_vel_pub = rospy.Publisher('/{}/turtle1/cmd_vel'.format(ns1), Twist, queue_size=5)
            self.subscriber = rospy.Subscriber("/{}/turtle1/pose".format(ns1), Pose, self.pose_callback)
            self.subscriber_target = rospy.Subscriber("/targets_topic", Pose, self.target_callback)
        
        if ns2 in ns:
            self.cmd_vel_pub = rospy.Publisher('/{}/turtle1/cmd_vel'.format(ns2), Twist, queue_size=5)
            self.subscriber = rospy.Subscriber("/{}/turtle1/pose".format(ns2), Pose, self.pose_callback)
            self.subscriber_target = rospy.Subscriber("/{}/turtle1/pose".format(ns1), Pose, self.target_callback)

    def init_coords(self):
        self.x = 0
        self.y = 0

        self.target_x = 0
        self.target_y = 0
        
    def target_callback(self, msg:Pose):
        # rospy.loginfo("target updated with: {0} {1}".format(msg.x, msg.y))
        self.target_x, self.target_y = msg.x, msg.y

    def pose_callback(self, msg:Pose):
        # rospy.loginfo("curr pose: {0} {1}".format(msg.x, msg.y))
        self.update_control(msg.x, msg.y)

    def update_control(self, x, y):
        self.x, self.y = x, y

    def spin(self):
        while not rospy.is_shutdown():
            twist_msg = Twist()
            
            diff_x = self.x - self.target_x
            diff_y = self.y - self.target_y

            if abs(diff_x) > 0.1:
                twist_msg.linear.x = -1.0 if diff_x > 0 else 1.0
            else:
                twist_msg.linear.x = 0.0

            if abs(diff_y) > 0.1:
                twist_msg.linear.y = -1.0 if diff_y > 0 else 1.0
            else:
                twist_msg.linear.y = 0.0

            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    sc = SimpleController()
    sc.spin()