#!/usr/bin/env python

import rospy
import math as m
from geometry_msgs.msg import Pose, Twist, Vector3, Pose2D
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import sign
import rosbag

class Square:
    def __init__(self, bag_name = 'test'):
        self.bag = rosbag.Bag(bag_name, 'w')

        self.path_subscriber = rospy.Subscriber("/path", Path, self.callback_set_vel)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.callback_read_odom)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # needed so ros has time to get self.current_x for two publishers
        rospy.sleep(1)
        self.error_pub = rospy.Publisher("/error", Pose2D, queue_size=10)
        self.gazebo_odom_subscriber = rospy.Subscriber("/gazebo_odom", Odometry, self.callback_read_gazebo_odom)

        self.freq = 10
        self.rate = rospy.Rate(self.freq) # 10hz

    def callback_read_odom(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orient = msg.pose.pose.orientation
        _,_,self.current_theta = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

    def callback_read_gazebo_odom(self, msg):
        self.ideal_x = msg.pose.pose.position.x
        self.ideal_y = msg.pose.pose.position.y
        ideal_orient = msg.pose.pose.orientation
        _,_,self.ideal_theta = euler_from_quaternion([ideal_orient.x, ideal_orient.y, ideal_orient.z, ideal_orient.w])
        
        error = Pose2D()
        error.x = self.current_x - self.ideal_x
        error.y= self.current_y - self.ideal_y
        error.theta = self.current_theta - self.ideal_theta
        self.error_pub.publish(error)

        # writing to a bag file
        self.bag.write('/gazebo_odom', Pose2D(self.ideal_x, self.ideal_y, self.ideal_theta))
        self.bag.write('/odometry', Pose2D(self.current_x,self.current_y, self.current_theta))
        self.bag.write('/error', error)

    def callback_set_vel(self, msg):
        new_msg = Twist()

        omega = 0.15
        vel = 0.25

        path = msg.poses
        stop_msg = Twist()

        for dest in path:  
            last_direct = 1000000
            last_dist = 100000

            x = dest.pose.position.x
            y = dest.pose.position.y

            dx = x - self.current_x
            dy = y - self.current_y

            direct = m.atan2(dy, dx) - self.current_theta	 

            last_direct = direct

            while True: 
                new_msg.angular.z = self.get_vel_sign(direct, omega)

                self.pub.publish(new_msg)
                self.rate.sleep()

                last_direct = direct
                
                direct = m.atan2(dy, dx) - self.current_theta

                # cast to (-pi:pi) range
                if direct < -m.pi: 
                    direct += 2*m.pi
                elif direct > m.pi: 
                    direct -=2*m.pi

                if (abs(direct) < 0.1) and (abs(direct) > abs(last_direct)):
                    break

            new_msg.angular.z = 0
            dx = x - self.current_x
            dy = y - self.current_y
            dist = m.sqrt(dx*dx + dy*dy)
            while abs(dist) <= abs(last_dist):

                new_msg.linear.x = vel
                self.pub.publish(new_msg)
                self.rate.sleep()

                last_dist = dist

                dx = x - self.current_x
                dy = y - self.current_y

                dist = m.sqrt(dx*dx + dy*dy)

            new_msg.linear.x = 0

        last_direct = 1000000 
        direct = - self.current_theta

        # obrot do pozycji poczatkowej
        while True:
            new_msg.angular.z = self.get_vel_sign(direct, omega)

            self.pub.publish(new_msg)
            self.rate.sleep()

            last_direct = direct
            
            direct = -self.current_theta

            # cast to (-pi:pi) range
            if direct < -m.pi: 
                direct += 2*m.pi
            elif direct > m.pi: 
                direct -=2*m.pi

            if (abs(direct) < 0.1) and (abs(direct) > abs(last_direct)):
                break

        new_msg.angular.z = 0
        self.pub.publish(new_msg)
        self.rate.sleep()
        self.bag.close()
        
    def get_vel_sign(self,direct, omega):
        
        if direct<0 and direct>-m.pi:              ###TODO 4 cwiartki
            z = -omega

        elif direct < -m.pi and direct > -2*m.pi:
            z = omega

        elif direct > 0 and direct < m.pi:
            z = omega

        elif direct > m.pi and direct < 2*m.pi:
            z = -omega

        else:
            z = omega
        
        return z

if __name__ == '__main__':
    rospy.init_node('Square_test')
    Square('TestK_L_C_T.bag')
    rospy.spin()
