#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Path_gen:
    def __init__(self, dir=1):
        pub = rospy.Publisher("/path", Path, queue_size=10)
        mssg = Path()
        P1 = PoseStamped()
        P2 = PoseStamped()
        P3 = PoseStamped()
        P4 = PoseStamped()

        a = 2  # square side length
        P1.pose.position.x = dir*a
        mssg.poses.append(P1)

        P2.pose.position.x = dir*a
        P2.pose.position.y = -a
        mssg.poses.append(P2)

        P3.pose.position.y = -a
        mssg.poses.append(P3)

        mssg.poses.append(P4)
        rospy.sleep(3)
        pub.publish(mssg)
        rospy.loginfo("Path sent")


if __name__ == '__main__':
    rospy.init_node('Path')
    Path_gen(dir=1)  # 1 - left, -1 right
    pub = rospy.Publisher("/path", Path, queue_size=10)

    rospy.spin()
