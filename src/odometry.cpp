#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <random>
#include <omnivelma_msgs/Vels.h>
#include <omnivelma_msgs/EncodersStamped.h>
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"

class CalcOdom
{
private:
    ros::Publisher pub;
    ros::Subscriber number_subscriber;
    float wheelRadius = 0.1 * 1.0164; // value from odom correction calculation;
    float modelWidth = 0.76;
    float modelLength = 0.72;
    float flVel = 0;
    float frVel = 0;
    float rlVel = 0;
    float rrVel = 0;

    float prev_x = 0;
    float prev_y = 0;
    float prev_th = 0;

    int counter = 0;

    float prev_time = 0;

    bool time_initialized = false;
    tf::Quaternion odom_quat;
    float euler_z, euler_old;
    nav_msgs::Odometry odometryMsg;
    int delay = 0;

    // gaussian noise
    std::default_random_engine generator;
    double var_ = 0.01;
    double sum_x = 0, sum_y = 0, sum_th = 0;

public:
    CalcOdom(ros::NodeHandle *nh)
    {
        pub = nh->advertise<nav_msgs::Odometry>("/odom/noisy", 10);
        number_subscriber = nh->subscribe("/omnivelma/encoders", 100, &CalcOdom::callback_vels, this);
    }
    void callback_vels(const omnivelma_msgs::EncodersStamped &encoders)
    {
        //publish every 20th message
        delay++;
        if (delay % 20 != 0)
        {
            return;
        }
        delay = 0;
        omnivelma_msgs::Vels msg = encoders.encoders.velocity;

        // add gaussian noise
        std::normal_distribution<double> distribution_fr(msg.fr, var_);
        std::normal_distribution<double> distribution_fl(msg.fl, var_);
        std::normal_distribution<double> distribution_rr(msg.rr, var_);
        std::normal_distribution<double> distribution_rl(msg.rl, var_);

        msg.fr = distribution_fr(generator);
        msg.fl = distribution_fl(generator);
        msg.rr = distribution_rr(generator);
        msg.rl = distribution_rl(generator);

        // end preprocessing

        odometryMsg.header.seq = counter;
        odometryMsg.header.stamp = ros::Time::now();
        odometryMsg.header.frame_id = "odom";
        odometryMsg.child_frame_id = "base_footprint";
        double velX = -msg.fr + msg.fl - msg.rl + msg.rr;
        double velY = msg.fr + msg.fl + msg.rl + msg.rr;

        velX *= 0.25 * wheelRadius;
        velY *= 0.25 * wheelRadius;
        float temp;
        temp = velX;
        velX = velY;
        velY = -temp;
        double k = 2.0 / (modelWidth + modelLength);
        double rot = msg.fr - msg.fl - msg.rl + msg.rr;
        rot *= k * 0.25 * wheelRadius;

        if (!time_initialized)
        {
            prev_time = odometryMsg.header.stamp.toSec();
            time_initialized = true;
        }
        auto dt = odometryMsg.header.stamp.toSec() - prev_time;

        odometryMsg.twist.twist.linear.x = velX;
        odometryMsg.twist.twist.linear.y = velY;
        odometryMsg.twist.twist.angular.z = rot;
        odometryMsg.pose.pose.position.x = prev_x + dt * velX * std::cos(prev_th) - dt * velY * std::sin(prev_th);
        odometryMsg.pose.pose.position.y = prev_y + dt * velX * std::sin(prev_th) + dt * velY * std::cos(prev_th);
        odom_quat.setRPY(0, 0, prev_th + rot * dt);
        prev_x += dt * velX * std::cos(prev_th) - dt * velY * std::sin(prev_th);
        prev_y += dt * velX * std::sin(prev_th) + dt * velY * std::cos(prev_th);
        prev_th += rot * dt;

        odometryMsg.pose.pose.orientation.x = odom_quat.x();
        odometryMsg.pose.pose.orientation.y = odom_quat.y();
        odometryMsg.pose.pose.orientation.z = odom_quat.z();
        odometryMsg.pose.pose.orientation.w = odom_quat.w();

        pub.publish(odometryMsg);

        prev_time = odometryMsg.header.stamp.toSec();
        counter++;
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "false");
    ros::NodeHandle nh;
    CalcOdom odom_noise = CalcOdom(&nh);
    ros::spin();
}