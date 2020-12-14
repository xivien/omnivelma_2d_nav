#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
/*
Hector mapping publishes in a covariance field a Hessian - it won't work with robot
localization package, it needs to be changed to custom covariance values
*/
class MessageCov
{
public:
  MessageCov()
  {
    sub_imu = n_.subscribe("/imu/filtered", 1000, &MessageCov::ImuCallback, this);
    pub_imu = n_.advertise<sensor_msgs::Imu>("/imu_cov", 10);
  }

  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_ptr)
  {
    sensor_msgs::Imu imu;
    imu = *imu_ptr;

    // imu.orientation_covariance[8] = 0.001;
    // imu.angular_velocity_covariance[8] = 1e-6;
    // imu.linear_acceleration_covariance[0] = 1e-5;
    // imu.linear_acceleration_covariance[4] = 1e-5;

    pub_imu.publish(imu);
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_imu;
  ros::Publisher pub_imu;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "message_cov_pub");  // Init ROS
  MessageCov Mc;                             // Construct class
  ros::spin();                               // Run until interupted
  return 0;
};