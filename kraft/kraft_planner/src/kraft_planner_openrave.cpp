#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/GetJointProperties.h>
#include "gazebo_msgs/ModelState.h"
#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>

// OpenRAVE
//#include <openrave-core.h>

ros::Publisher SA_pub, SE_pub, tr_pub, WP_pub, WY_pub, WR_pub, pose_publisher, gizmo_pub;
boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> group_;

void getGroupInfo(){
	geometry_msgs::PoseStamped currentPose = group_->getCurrentPose();
	printf("x: %f\n", currentPose.pose.position.x);
	printf("y: %f\n", currentPose.pose.position.y);
	printf("z: %f\n", currentPose.pose.position.z);
	printf("\n");
	printf("x: %f\n", currentPose.pose.orientation.x);
	printf("y: %f\n", currentPose.pose.orientation.y);
	printf("z: %f\n", currentPose.pose.orientation.z);
	printf("w: %f\n", currentPose.pose.orientation.w);
	printf("\n");
}

void moveGroup(const geometry_msgs::PoseStamped::ConstPtr& msg){
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::planning_interface::MoveItErrorCode success;
	geometry_msgs::Pose target_pose;
	geometry_msgs::PoseStamped currentPose = group_->getCurrentPose();

	tf::StampedTransform transform;
    tf::TransformListener listener;
    listener.waitForTransform("/phantom/base", "/phantom/stylus", ros::Time::now(), ros::Duration(0.25));
    try {
      listener.lookupTransform("/phantom/base", "/phantom/stylus", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::TransformStamped tfmsg;
    tf::transformStampedTFToMsg(transform, tfmsg);

    target_pose.position.x = tfmsg.transform.translation.x + tfmsg.transform.translation.x * 2;
    target_pose.position.y = tfmsg.transform.translation.y + tfmsg.transform.translation.y * 2;// + 0.53;
    target_pose.position.z = tfmsg.transform.translation.z + tfmsg.transform.translation.z * 2 + 0.82;
    target_pose.orientation.x = tfmsg.transform.rotation.x;
    target_pose.orientation.y = tfmsg.transform.rotation.y;
    target_pose.orientation.z = tfmsg.transform.rotation.z;
    target_pose.orientation.w = tfmsg.transform.rotation.w;

    double roll, pitch, yaw;
    tf::Quaternion q(tfmsg.transform.rotation.x, 
        tfmsg.transform.rotation.y, 
        tfmsg.transform.rotation.z, 
        tfmsg.transform.rotation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //roll -= M_PI/2;
    yaw += M_PI;

    tf::Quaternion q2 = tf::createQuaternionFromRPY(roll, pitch, yaw);

    target_pose.orientation.x = q2[0];
    target_pose.orientation.y = q2[1];
    target_pose.orientation.z = q2[2];
    target_pose.orientation.w = q2[3];

/*
	target_pose.position.x = currentPose.pose.position.x;
	target_pose.position.y = currentPose.pose.position.y;
	target_pose.position.z = currentPose.pose.position.z;
*
	target_pose.orientation.x = currentPose.pose.orientation.x;
	target_pose.orientation.y = currentPose.pose.orientation.y;
	target_pose.orientation.z = currentPose.pose.orientation.z;
	target_pose.orientation.w = currentPose.pose.orientation.w;

	target_pose.position.x = 0.01;
	target_pose.position.y = 0.51;
	target_pose.position.z = 0.45;
/*
	target_pose.orientation.x = 0.0;
	target_pose.orientation.y = 0.0;
	target_pose.orientation.z = 0.0;
	target_pose.orientation.w = 1.0;
*/

	//target_pose.position.x += 0.05;
	//target_pose.position.y += 0.1;
	//target_pose.position.z -= 0.05;

	group_->setGoalOrientationTolerance(1.0);
	group_->setGoalPositionTolerance(0.05);
	printf("Orient Tolerance: %f\n", group_->getGoalOrientationTolerance());
	printf("Posit Tolerance: %f\n", group_->getGoalPositionTolerance());

	group_->setPoseTarget(target_pose);	
	success = group_->plan(my_plan);
	if (success == moveit_msgs::MoveItErrorCodes::SUCCESS){
		ROS_INFO("SUCCESS");
		group_->execute(my_plan);
		//group_->asyncExecute(my_plan);
	}else{
		ROS_INFO("NO SUCCESS");
	}
	
}

void moveGizmoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    gazebo_msgs::ModelState gizmopos;
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success;
	geometry_msgs::Pose target_pose;
	//geometry_msgs::PoseStamped currentPose = group_->getCurrentPose();

    tf::StampedTransform transform;
    tf::TransformListener listener;
    listener.waitForTransform("/phantom/base", "/phantom/stylus", ros::Time::now(), ros::Duration(0.25));
    try {
      listener.lookupTransform("/phantom/base", "/phantom/stylus", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Convert tf::StampedTransform to TransformStamped msg
    geometry_msgs::TransformStamped tfmsg;
    tf::transformStampedTFToMsg(transform, tfmsg);

    gizmopos.model_name = "test_axis";
    gizmopos.pose.position.x = tfmsg.transform.translation.x + tfmsg.transform.translation.x * 2;
    gizmopos.pose.position.y = tfmsg.transform.translation.y + tfmsg.transform.translation.y * 2;// + 0.53;
    gizmopos.pose.position.z = tfmsg.transform.translation.z + tfmsg.transform.translation.z * 2 + 0.82;
    gizmopos.pose.orientation.x = tfmsg.transform.rotation.x;
    gizmopos.pose.orientation.y = tfmsg.transform.rotation.y;
    gizmopos.pose.orientation.z = tfmsg.transform.rotation.z;
    gizmopos.pose.orientation.w = tfmsg.transform.rotation.w;

    printf("position.x: [%f]\n",gizmopos.pose.position.x);
    printf("position.y: [%f]\n",gizmopos.pose.position.y);
    printf("position.z: [%f]\n",gizmopos.pose.position.z);
    printf("----\n");
    printf("orientation.x: [%f]\n",gizmopos.pose.orientation.x);
    printf("orientation.y: [%f]\n",gizmopos.pose.orientation.y);
    printf("orientation.z: [%f]\n",gizmopos.pose.orientation.z);
    printf("orientation.w: [%f]\n",gizmopos.pose.orientation.w);

    double roll, pitch, yaw;
    tf::Quaternion q(tfmsg.transform.rotation.x, 
        tfmsg.transform.rotation.y, 
        tfmsg.transform.rotation.z, 
        tfmsg.transform.rotation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //roll -= M_PI/2;
    yaw += M_PI;

    tf::Quaternion q2 = tf::createQuaternionFromRPY(roll, pitch, yaw);

    gizmopos.pose.orientation.x = q2[0];
    gizmopos.pose.orientation.y = q2[1];
    gizmopos.pose.orientation.z = q2[2];
    gizmopos.pose.orientation.w = q2[3];

    gizmo_pub.publish(gizmopos);

    printf("END MOVECUBE\n");
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "kraft_planner_openrave");
	ros::NodeHandle n, m, nh_SA, nh_SE, nh_tr, nh_WP, nh_WY, nh_WR, nh_gizmo;
	ros::NodeHandle node("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	printf("A\n");
	SA_pub = nh_SA.advertise<std_msgs::Float64>("/SA/command", 1000);
	SE_pub = nh_SE.advertise<std_msgs::Float64>("/SE/command", 1000);
	tr_pub = nh_tr.advertise<std_msgs::Float64>("/linkage_tr/command", 1000);
	WP_pub = nh_WP.advertise<std_msgs::Float64>("/WP/command", 1000);
	WY_pub = nh_WY.advertise<std_msgs::Float64>("/WY/command", 1000);
	WR_pub = nh_WR.advertise<std_msgs::Float64>("/WR/command", 1000);
    //gizmo_pub = nh_gizmo.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1000);
    //ros::Subscriber gizmo_sub = m.subscribe("/phantom/pose", 100, moveGizmoCallback);

    printf("B\n");

	group_.reset(new moveit::planning_interface::MoveGroupInterface("kraft"));
	printf("C\n");
	group_->setPlanningTime(1.0);
	printf("D\n");
	ros::Subscriber gizmo_sub = m.subscribe("/phantom/pose", 100, moveGroup);
	//getGroupInfo();
	//moveGroup();
	//ros::waitForShutdown();
	ros::spin();
	return 0;
}