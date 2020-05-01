#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveBaseController
{
public:
  MoveBaseController() : ac_("move_base", true)
  {
    pub_init_pose_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    sub = n_.subscribe("/amcl_pose", 1, &MoveBaseController::msgCallback, this);
    ROS_INFO("MAKE SURE THAT ROBOT HAS AN EMPTY 1mX1m AREA IN FRONT OF IT");
    ROS_INFO("Select mode: \n"
             "0 - reset localization and do homing sequence \n"
             "1 - set manual localization and do homing sequence \n"
             "2 - no homing sequence start with localization 0.0 \n");
    ros::Duration(1.0).sleep();
    std::cin >> mode_;

    freq_ = 10.0f;
    ros::Rate loop_rate(freq_);
    // reset AMCL particles
    if (mode_ == 0)
    {
      ROS_INFO("Resetting AMCL POSE");
      g_l_client_ = n_.serviceClient<std_srvs::Empty>("/global_localization");
      std_srvs::Empty emptySrv;
      g_l_client_.call(emptySrv);
      ros::Duration(1.0).sleep();
    }
    // Set new starting position
    else if (mode_ == 1)
    {
      geometry_msgs::PoseWithCovarianceStamped init_pose;
      ros::Duration(1.0).sleep();
      init_pose.header.frame_id = "map";
      init_pose.pose.pose.orientation.w = 1;
      ROS_INFO("Enter X[m] Y[m] Theta[rad]");
      ros::Duration(1.0).sleep();
      std::cin >> init_pose.pose.pose.position.x >> init_pose.pose.pose.position.y >> init_pose.pose.pose.orientation.z;

      // init covariances
      init_pose.pose.covariance[0] = 0.25;
      init_pose.pose.covariance[7] = 0.25;
      init_pose.pose.covariance[14] = 0.0625;
      ROS_INFO("Setting pose to x=%.2f y=%.2f theta=%.2f", init_pose.pose.pose.position.x,
               init_pose.pose.pose.position.y, init_pose.pose.pose.orientation.z);
      ros::Duration(1.0).sleep();
      pub_init_pose_.publish(init_pose);
      ros::spinOnce();
    }
    // do homing sequence
    if (mode_ < 2)
    {
      float vel = 0.2f;
      float omega = 0.25f;
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
      if (cov_x_ > cov_tol_ || cov_y_ > cov_tol_)
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

    // check quality of localization
    ros::spinOnce();
    if (cov_x_ > cov_tol_ || cov_y_ > cov_tol_)
    {
      ROS_WARN("High covariance \n The robot may crash \n"
               "Aborting navigation");
      ros::Duration(10.0).sleep();
    }
    // Send navigation goal
    else
    {
      while (true)
      {
        ROS_INFO("To exit use invalid input");
        ROS_INFO("Send navigation goal X[m] Y[m] Theta[rad]");
        ros::Duration(1.0).sleep();
        std::cin >> goal_x_ >> goal_y_ >> goal_theta_;
        if (!std::cin)
        {
          ROS_INFO("Exiting \n Press ctrl+c to quit the program");
          break;
        }

        ROS_INFO("Navigating to a goal x=%.2f y=%.2f theta=%.2f", goal_x_, goal_y_, goal_theta_);
        send_goal();
      }
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
    goal_.target_pose.pose.orientation.z = goal_theta_;
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
  ros::Publisher pub, pub_init_pose_;
  ros::Subscriber sub;
  MoveBaseClient ac_;
  move_base_msgs::MoveBaseGoal goal_;
  ros::ServiceClient g_l_client_;

  float freq_;
  int mode_;
  float goal_x_, goal_y_, goal_theta_;
  double cov_x_, cov_y_;
  float cov_tol_{ 0.1 };
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveBaseController");  // Init ROS
  MoveBaseController MoveBaseCon;               // Construct class
  ros::spin();                                  // Run until interupted
  return 0;
}