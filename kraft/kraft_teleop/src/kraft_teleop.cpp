#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "gazebo_msgs/GetJointProperties.h"
#include <sstream>
#include <string>
#include <iostream>

ros::Publisher SA_pub, SE_pub, linkage_tr_pub, WP_pub, WY_pub, WR_pub;
ros::Subscriber SA_sub;

ros::ServiceClient client;

gazebo_msgs::GetJointProperties jointstate;;

float getjointstate(std::string jointname)
{
	float jointposition;
	jointstate.request.joint_name = jointname;
	client.call(jointstate);
	jointposition = jointstate.response.position[0];
	ROS_INFO("%s state: %f", jointname.c_str(), jointposition);
	
	return jointposition;
}

void userManual()
{
	ROS_WARN("Joint SA: a-d");
	ROS_WARN("Joint SE: s-w"); 
	ROS_WARN("Joint linkage_tr: f-h");
	ROS_WARN("Joint WP: g-t");
	ROS_WARN("Joint WY: j-l");
	ROS_WARN("Joint WR: k-i");
	ROS_WARN("Print manual: m");
}

bool manipulate()
{
	std_msgs::Float64 msg;
	char c = getchar();
	switch (c) {
		case 'a':
			msg.data = getjointstate("kraft::SA") - 0.1f;
			SA_pub.publish(msg);
			break;
    		case 'd':
			msg.data = getjointstate("kraft::SA") + 0.1f;
			SA_pub.publish(msg);
			break;
    		case 's':
			msg.data = getjointstate("kraft::SE") - 0.1f;
			SE_pub.publish(msg);
			break;
    		case 'w':
			msg.data = getjointstate("kraft::SE") + 0.1f;
			SE_pub.publish(msg);
			break;


		case 'f':
			msg.data = getjointstate("kraft::linkage_tr") - 0.1f;
			linkage_tr_pub.publish(msg);
			break;
    		case 'h':
			msg.data = getjointstate("kraft::linkage_tr") + 0.1f;
			linkage_tr_pub.publish(msg);
			break;
    		case 'g':
			msg.data = getjointstate("kraft::WP") - 0.1f;
			WP_pub.publish(msg);
			break;
    		case 't':
			msg.data = getjointstate("kraft::WP") + 0.1f;
			WP_pub.publish(msg);
			break;


		case 'j':
			msg.data = getjointstate("kraft::WY") - 0.1f;
			WY_pub.publish(msg);
			break;
    		case 'l':
			msg.data = getjointstate("kraft::WY") + 0.1f;
			WY_pub.publish(msg);
			break;
    		case 'k':
			msg.data = getjointstate("kraft::WR") - 0.1f;
			WR_pub.publish(msg);
			break;
    		case 'i':
			msg.data = getjointstate("kraft::WR") + 0.1f;
			WR_pub.publish(msg);
			break;

    		case 'm':
			userManual();
			break;
		case '\n':
			break;
		case ' ':
			return false;
		default:
			ROS_WARN("Tecla invalida.");
			return true;
	}
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "kraft_teleop");
	ros::NodeHandle nh_SA, nh_SE, nh_linkage_tr, nh_WP, nh_WY, nh_WR, nh_client;
	SA_pub = nh_SA.advertise<std_msgs::Float64>("/SA/command", 1000);
	SE_pub = nh_SE.advertise<std_msgs::Float64>("/SE/command", 1000);
	linkage_tr_pub = nh_linkage_tr.advertise<std_msgs::Float64>("/linkage_tr/command", 1000);
	WP_pub = nh_WP.advertise<std_msgs::Float64>("/WP/command", 1000);
	WY_pub = nh_WY.advertise<std_msgs::Float64>("/WY/command", 1000);
	WR_pub = nh_WR.advertise<std_msgs::Float64>("/WR/command", 1000);

	client = nh_client.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

	while (ros::ok())
	{
		while(manipulate()){
			;
		}
	}

	return 0;
}

