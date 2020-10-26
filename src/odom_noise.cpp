#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <random>

class OdomNoise
{
private:
  ros::Publisher pub;
  ros::Subscriber number_subscriber;

public:
  OdomNoise(ros::NodeHandle *nh)
  {
    pub = nh->advertise<nav_msgs::Odometry>("/odom/noisy", 10);
    number_subscriber = nh->subscribe("/odom/throttled", 1000, &OdomNoise::callback_odom, this);
  }
  void callback_odom(const nav_msgs::Odometry &odom)
  {
    nav_msgs::Odometry odom_noisy;
    odom_noisy = odom;

    odom_noisy.pose.covariance[0] = 0.01;
    odom_noisy.pose.covariance[7] = 0.01;
    odom_noisy.pose.covariance[35] = 0.1;

    odom_noisy.twist.covariance[0] = 0.002;
    odom_noisy.twist.covariance[7] = 0.002;
    odom_noisy.twist.covariance[35] = 0.01;
    pub.publish(odom_noisy);
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_noise");
  ros::NodeHandle nh;
  OdomNoise odom_noise = OdomNoise(&nh);
  ros::spin();
}