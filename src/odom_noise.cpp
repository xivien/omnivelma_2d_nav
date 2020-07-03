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
  double sum_x = 0, sum_y = 0, sum_th = 0;

public:
  OdomNoise(ros::NodeHandle *nh)
  {
    var_ = 0.01;
    pub = nh->advertise<nav_msgs::Odometry>("/odom/noisy", 10);
    number_subscriber = nh->subscribe("/odom/throttled", 1000, &OdomNoise::callback_odom, this);
  }
  void callback_odom(const nav_msgs::Odometry &odom)
  {
    nav_msgs::Odometry odom_noisy;
    std::normal_distribution<double> distribution_vx(odom.twist.twist.linear.x, var_);
    std::normal_distribution<double> distribution_vy(odom.twist.twist.linear.y, var_);
    std::normal_distribution<double> distribution_vth(odom.twist.twist.angular.z, var_);
    // std::normal_distribution<double> distribution_x(0.0001, var_);
    // std::normal_distribution<double> distribution_y(0.0001, var_);
    // std::normal_distribution<double> distribution_th(0.0001, var_);
    odom_noisy = odom;

    // sum_x += distribution_x(generator);
    // sum_y += distribution_y(generator);
    // sum_th += distribution_th(generator);

    // odom_noisy.twist.twist.linear.x = distribution_vx(generator);
    // odom_noisy.twist.twist.linear.y = distribution_vy(generator);
    // odom_noisy.twist.twist.angular.y = distribution_vth(generator);

    // odom_noisy.pose.pose.position.x += sum_x;
    // odom_noisy.pose.pose.position.y += sum_y;
    // odom_noisy.pose.pose.orientation.z += sum_th;

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