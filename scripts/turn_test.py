#!/usr/bin/env python


import rospy
import math as m
from geometry_msgs.msg import Pose, Twist, Vector3, Pose2D
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import rosbag


class turn_test:
    def __init__(self, bag_name='turn_test.bag'):
        self.bag = rosbag.Bag(bag_name, 'w')

        self.current_theta = 0
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
        orient = msg.pose.pose.orientation
        _, _, self.current_theta = euler_from_quaternion(
            [orient.x, orient.y, orient.z, orient.w])

    def callback_read_gazebo_odom(self, msg):
        ideal_orient = msg.pose.pose.orientation
        _, _, self.ideal_theta = euler_from_quaternion(
            [ideal_orient.x, ideal_orient.y, ideal_orient.z, ideal_orient.w])

        error = Pose2D()
        error.theta = self.current_theta - self.ideal_theta
        self.error_pub.publish(error)

        # writing to a bag file
        self.bag.write('/gazebo_odom', Pose2D(0, 0, self.ideal_theta))
        self.bag.write('/odometry', Pose2D(0, 0, self.current_theta))
        self.bag.write('/error', error)

    def callback_set_vel(self, msg):
        new_msg = Twist()

        omega = 0.5

        spinned = 0
        desired = msg.data

        new_msg.angular.z = omega

        last_theta = self.ideal_theta

        while (self.ideal_theta + spinned) < desired:

            self.pub.publish(new_msg)
            self.rate.sleep()

            if self.current_theta < last_theta:
                spinned += 2 * m.pi

            last_theta = self.ideal_theta

        new_msg.angular.z = 0
        self.pub.publish(new_msg)
        self.rate.sleep()
        self.bag.close()


if __name__ == '__main__':
    rospy.init_node('turn_test')
    turn_test('NEWBAG.bag')
    rospy.spin()
