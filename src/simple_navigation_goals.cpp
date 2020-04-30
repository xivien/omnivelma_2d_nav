#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveBaseController
{
public:
  MoveBaseController(int i) : ac_("move_base", true), mode_(i)
  {
    goal_x_ = 1;
    goal_y_ = -1;

    pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    sub = n_.subscribe("/amcl_pose", 1, &MoveBaseController::msgCallback, this);

    freq_ = 10.0f;
    ros::Rate loop_rate(freq_);

    if (mode_ != 0)
    {
      ROS_INFO("Resetting AMCL POSE");
      g_l_client_ = n_.serviceClient<std_srvs::Empty>("/global_localization");

      // reset AMCL particles
      std_srvs::Empty emptySrv;
      g_l_client_.call(emptySrv);
      ros::Duration(1.0).sleep();

      float vel = 0.2f;
      float omega = 0.2f;
      float dist = 1.0f;

      int mov_steps = freq_ * dist / vel;
      int rot_steps = freq_ * 2 * M_PI / omega;

      geometry_msgs::Twist new_msg;

      std::array<int, 4> dir_x = { 1, 0, -1, 0 };
      std::array<int, 4> dir_y = { 0, -1, 0, 1 };
      ROS_INFO("Homing sequence initiated \n Rotating");

      for (int i = 0; i < rot_steps; i++)
      {
        new_msg.angular.z = omega;
        pub.publish(new_msg);
        loop_rate.sleep();
      }

      new_msg.angular.z = 0.0f;
      pub.publish(new_msg);
      loop_rate.sleep();

      ros::spinOnce();
      if (cov_x_ > 0.1 || cov_y_ > 0.1)
      {
        ROS_INFO("High covariance, doing a square 1x1m trajectory");
        for (int i = 0; i < 4; i++)
        {
          for (int j = 0; j < mov_steps; j++)
          {
            new_msg.linear.x = vel * dir_x[i];
            new_msg.linear.y = vel * dir_y[i];
            pub.publish(new_msg);
            loop_rate.sleep();
          }
        }

        new_msg.linear.x = 0.0f;
        new_msg.linear.y = 0.0f;
        pub.publish(new_msg);
        loop_rate.sleep();
      }
    }

    ros::spinOnce();
    if (cov_x_ > 0.1 || cov_y_ > 0.1)
    {
      ROS_WARN("High covariance \n The robot may crash");
      ros::Duration(10.0).sleep();
    }
    else
    {
      ros::Duration(1.0).sleep();
      ROS_INFO("Navigating to a goal");
      send_goal();
    }
  }
  void send_goal()
  {
    while (!ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.header.stamp = ros::Time::now();

    goal_.target_pose.pose.position.x = goal_x_;
    goal_.target_pose.pose.position.y = goal_y_;
    goal_.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac_.sendGoal(goal_);

    ac_.waitForResult();

    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Travelling to goal");
    else
      ROS_INFO("The base failed to move");
  }

  void msgCallback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose)
  {
    cov_x_ = amcl_pose.pose.covariance[0];
    cov_y_ = amcl_pose.pose.covariance[7];
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub;
  ros::Subscriber sub;
  MoveBaseClient ac_;
  move_base_msgs::MoveBaseGoal goal_;
  ros::ServiceClient g_l_client_;
  float freq_;
  int mode_, goal_x_, goal_y_;
  double cov_x_, cov_y_;
};
int main(int argc, char** argv)
{
  int mode = 1;
  ros::init(argc, argv, "MoveBaseController");  // Init ROS
  MoveBaseController MoveBaseCon(mode);         // Construct class
  ros::spin();                                  // Run until interupted
  return 0;
}