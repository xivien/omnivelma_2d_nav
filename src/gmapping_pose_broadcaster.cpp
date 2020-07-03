#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

class PosePub
{
public:
  PosePub()
  {
    pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gmapping/pose", 10);
    sub = n_.subscribe("/odom/noisy", 1000, &PosePub::msgCallback, this);
  }

  void msgCallback(const nav_msgs::OdometryConstPtr& point_ptr)
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::PoseWithCovarianceStamped g_pose;
    g_pose.header.frame_id = "map";
    g_pose.header.stamp = point_ptr->header.stamp;
    g_pose.header.seq = point_ptr->header.seq;

    g_pose.pose.covariance = point_ptr->pose.covariance;
    g_pose.pose.pose.orientation = point_ptr->pose.pose.orientation;
    g_pose.pose.pose.position.x = transform.getOrigin().x();
    g_pose.pose.pose.position.y = transform.getOrigin().y();
    g_pose.pose.pose.position.z = transform.getOrigin().z();

    for (int i = 0; i < pose_covar.size(); i++)
    {
      g_pose.pose.covariance[i] = pose_covar.at(i);
    }



    pub.publish(g_pose);
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub;
  ros::Publisher pub;
  tf::TransformListener listener;


std::vector<double> pose_covar{ 0.1404, 0, 0, 0, 0, 0, 
                                  0, 0.1404, 0, 0, 0, 0, 
                                  0, 0, 0.1404, 0, 0, 0,
                                  0, 0, 0, 0.0004, 0, 0, 
                                  0, 0, 0, 0, 0.0004, 0, 
                                  0, 0, 0, 0, 0, 0.0004 };
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "g_pose");  // Init ROS
  PosePub gp;                       // Construct class
  ros::spin();                      // Run until interupted
  return 0;
};