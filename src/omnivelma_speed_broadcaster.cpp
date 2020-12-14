#include <iostream>
#include <string>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <cmath>
#include <ros/console.h>
#include <exception>
#include <omnivelma_msgs/Vels.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <boost/lexical_cast.hpp>

using boost::lexical_cast;

// BASED ON PIOTR WALAS CODE
/// Nadajnik
ros::Publisher publisher;
ros::Publisher forSmootherPublisher;
ros::Publisher odometryPublisher;

const double wheelRadius = 0.1 * 1.0164;
const double modelWidth = 0.76;
const double modelLength = 0.72;

ros::Duration velPublisherSleep(0.01);

std::mutex newVelLock;
bool velInit = false;

omnivelma_msgs::Vels vels;

///Tryb rotacji wejścia
enum Rotation
{
	No,
	Left90,
	Back,
	Right90,
	Rviz
};

Rotation rotation;

///Funkcja wywoływana na odbiór wiadomości
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	newVelLock.lock();

	double velForw;
	double velRight;
	double rotLeft = msg -> angular.z;

	//PW
	rotLeft /= 4.0;
	switch(rotation)
	{
	case No:
		velForw = msg -> linear.y;
		velRight = msg -> linear.x;
		break;
	case Left90:
		velForw = msg -> linear.x;
		velRight = -msg -> linear.y;
		break;
	case Back:
		velForw = -msg -> linear.y;
		velRight = -msg -> linear.x;
		break;
	case Right90:
		velForw = -msg -> linear.x;
		velRight = msg -> linear.y;
		break;
	case Rviz:
		velForw = msg -> linear.x;
		velRight = msg -> linear.y;
		break;
	}
	
	vels.fr = (velForw - velRight + (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
	vels.fl = (velForw + velRight - (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
	vels.rl = (velForw - velRight - (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
	vels.rr = (velForw + velRight + (modelLength + modelWidth) * rotLeft * 2) / wheelRadius;
	publisher.publish(vels);

	velInit = true;

	newVelLock.unlock();	
}

void sendVelConstantly()
{
	while(ros::ok())		
	{
		newVelLock.lock();

		if(velInit == true)
		{
			publisher.publish(vels);
		}
		
		newVelLock.unlock();

		velPublisherSleep.sleep();
	}		
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "omnivelma_speed_broadcaster");
	ros::NodeHandle n("~");

	std::string progName(argv[0]);
	
	std::string sendTopicName;
	n.param<std::string>("vel_send_topic", sendTopicName, "/omnivelma/vels");

	std::string receiveTopicName;
	n.param<std::string>("vel_receive_topic", receiveTopicName, "cmd_vel");

	std::string rot;
	n.param<std::string>("rotation", rot, "0");
	if(rot == "90")
		rotation = Rotation::Left90;
	else if(rot == "180")
		rotation = Rotation::Back;
	else if(rot == "270")
		rotation = Rotation::Right90;
	else if(rot == "0")
		rotation = Rotation::No;

	//vel
	ros::NodeHandle handle;
	ros::Subscriber sub = handle.subscribe<geometry_msgs::Twist>(receiveTopicName, 10, twistCallback);
	if(!sub)
	{
		ROS_FATAL_STREAM("Nie można stworzyć odbiornika " + receiveTopicName);
	}
	publisher = handle.advertise<omnivelma_msgs::Vels>(sendTopicName, 10);
	if(!publisher)
	{
		ROS_FATAL_STREAM("Nie można stworzyć nadajnika " + sendTopicName);
		return -2;
	}

	//start cmd_vel spawn constantly
	std::thread c(sendVelConstantly);
	c.detach();

	ROS_WARN("vels will be published constantly!");
	ROS_WARN_STREAM("Choosen axis rotation: " + rot);

	ros::spin();

	return 0;
}
