#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <random>

class OdomNoise
{
private:
  ros::Publisher pub;
  ros::Subscriber number_subscriber;
  std::default_random_engine generator;
  double var_;

public:
  OdomNoise(ros::NodeHandle *nh)
  {
    var_ = 0.05;
    pub = nh->advertise<nav_msgs::Odometry>("/odom/noisy", 10);
    number_subscriber = nh->subscribe("/odom/throttled", 1000, &OdomNoise::callback_odom, this);
  }
  void callback_odom(const nav_msgs::Odometry &odom)
  {
    nav_msgs::Odometry odom_noisy;
    std::normal_distribution<double> distribution_x(odom.pose.pose.position.x, var_);
    std::normal_distribution<double> distribution_y(odom.pose.pose.position.y, var_);
    odom_noisy = odom;
    odom_noisy.pose.pose.position.x = distribution_x(generator);
    odom_noisy.pose.pose.position.y = distribution_y(generator);

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