#!/usr/bin/env python

# /elektron/mobile_base_controller/odom

import rospy
import math as m
from geometry_msgs.msg import Pose, Twist, Vector3, Pose2D
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import rosbag


class straight_test:
    def __init__(self, bag_name='straight_test.bag'):
        self.bag = rosbag.Bag(bag_name, 'w')

        self.target_subscriber = rospy.Subscriber(
            "/target", Float32, self.callback_set_vel)
        self.odom_subscriber = rospy.Subscriber(
            "/odom/filtered", Odometry, self.callback_read_odom)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # needed so ros has time to get self.current_x
        rospy.sleep(1)
        self.error_pub = rospy.Publisher("/error", Pose2D, queue_size=10)
        self.gazebo_odom_subscriber = rospy.Subscriber(
            "/PoseGlob", Odometry, self.callback_read_gazebo_odom)

        self.freq = 10
        self.rate = rospy.Rate(self.freq)  # 10hz

    def callback_read_odom(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def callback_read_gazebo_odom(self, msg):

        self.ideal_x = msg.pose.pose.position.x
        self.ideal_y = msg.pose.pose.position.y

        error = Pose2D()
        error.x = self.current_x - self.ideal_x
        error.y = self.current_y - self.ideal_y
        self.error_pub.publish(error)

        # writing to a bag file
        self.bag.write('/gazebo_odom', Pose2D(self.ideal_x, self.ideal_y, 0))
        self.bag.write('/odometry', Pose2D(self.current_x, self.current_y, 0))
        self.bag.write('/error', error)

    def callback_set_vel(self, msg):
        new_msg = Twist()

        vel = 0.25
        x = msg.data

        dist = m.sqrt((x-self.ideal_x)*(x-self.ideal_y))

        last_dist = 100000
        new_msg.linear.x = vel

        while abs(dist) <= abs(last_dist):

            self.pub.publish(new_msg)
            self.rate.sleep()

            last_dist = dist

            dx = x - self.ideal_x

            dist = m.sqrt(dx*dx)

        new_msg.linear.x = 0
        self.pub.publish(new_msg)
        self.rate.sleep()
        rospy.sleep(1)
        x = 0

        dist = m.sqrt(self.ideal_x*self.ideal_x)
        last_dist = 100000

        new_msg.linear.x = -vel

        while abs(dist) <= abs(last_dist):

            self.pub.publish(new_msg)
            self.rate.sleep()
            last_dist = dist

            dx = x - self.ideal_x

            dist = m.sqrt(dx*dx)

        new_msg.linear.x = 0
        self.pub.publish(new_msg)
        self.rate.sleep()
        self.bag.close()


if __name__ == '__main__':
    rospy.init_node('straight_test')
    straight_test()
    rospy.spin()
