#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

class PoseDrawer
{
public:
  PoseDrawer() : tf2_(buffer_), target_frame_("map"), tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    point_sub_.subscribe(n_, "/pose_with_covariance_stamped", 10);
    pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gmapping/pose", 10);
    tf2_filter_.registerCallback(boost::bind(&PoseDrawer::msgCallback, this, _1));
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  void msgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& point_ptr)
  {
    geometry_msgs::PoseWithCovarianceStamped point_out;
    try
    {
      buffer_.transform(*point_ptr, point_out, target_frame_);
      pub.publish(point_out);
      // ROS_INFO("pose in map frame(x:%f y:%f z:%f)\n", point_out.pose.pose.position.x, point_out.pose.pose.position.y,
      //          point_out.pose.pose.position.z);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Failure %s\n", ex.what());  // Print exception which was caught
    }
  }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped> tf2_filter_;

  ros::Publisher pub;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "g_pose");  // Init ROS
  PoseDrawer gp;                    // Construct class
  ros::spin();                      // Run until interupted
  return 0;
};