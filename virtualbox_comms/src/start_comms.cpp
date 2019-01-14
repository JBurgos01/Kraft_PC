#include "ros/ros.h"
#include "udp_client_server.h"
#include <virtualbox_comms/JointState.h>
#include <sstream>

void chatterCallback(const virtualbox_comms::JointState::ConstPtr& msg)
{
	// Do thingis
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "VB_comunicator");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<virtualbox_comms::JointState>("kraft_status", 1000);

	ros::Subscriber sub = n.subscribe("kraft_control", 1000, chatterCallback);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
/*		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;*/
	}

	return 0;
}