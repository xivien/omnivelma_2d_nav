#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
/*
Hector mapping publishes in a covariance field a Hessian - it won't work with robot
localization package, it needs to be changed to custom covariance values
*/
class HectorCov
{
public:
  HectorCov()
  {
    pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/hector_mapping/pose", 10);
    sub = n_.subscribe("/poseupdate", 1000, &HectorCov::msgCallback, this);
  }

  void msgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_ptr)
  {
    geometry_msgs::PoseWithCovarianceStamped h_pose;
    h_pose = *pose_ptr;

    for (int i = 0; i < pose_covar.size(); i++)
    {
      h_pose.pose.covariance[i] = pose_covar.at(i);
    }
    pub.publish(h_pose);
  }

private:
  ros::NodeHandle n_;
  //   ros::NodeHandle nh_private_("~");  // Private nodehandle for handling parameters
  ros::Subscriber sub;
  ros::Publisher pub;

  std::vector<double> pose_covar{ 0.1404, 0, 0, 0, 0, 0, 
                                  0, 0.1404, 0, 0, 0, 0, 
                                  0, 0, 0.1404, 0, 0, 0,
                                  0, 0, 0, 0.0004, 0, 0, 
                                  0, 0, 0, 0, 0.0004, 0, 
                                  0, 0, 0, 0, 0, 0.0004 };

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_cov_pub");  // Init ROS
  HectorCov gp;                             // Construct class
  ros::spin();                              // Run until interupted
  return 0;
};