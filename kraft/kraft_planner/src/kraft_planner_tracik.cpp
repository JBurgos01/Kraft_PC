#include <ros/ros.h>
#include <ros/param.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/ModelState.h"
#include <sstream>
#include <string>
#include <iostream>
#include <trac_ik/trac_ik.hpp>
#include <math.h>
#include <ros/duration.h>

#include <boost/date_time.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Geometry>
#include <limits>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include "tf_conversions/tf_kdl.h"
#include "angles/angles.h"

ros::Publisher SA_pub, SE_pub, tr_pub, WP_pub, WY_pub, WR_pub, pose_pub, gizmo_pub, real_pose_pub;
geometry_msgs::Pose end_effector_pose;
TRAC_IK::TRAC_IK *ik_solver;
KDL::Chain chain;
float sum;
int cnt, tries, tries_ok;
float phantom_x_min = -0.24;
float phantom_x_max = 0.24;
float phantom_y_min = 0.07;
float phantom_y_max = 0.27;
float phantom_z_min = -0.02;
float phantom_z_max = 0.32;
float kraft_x_min = -1.28;
float kraft_x_max = 1.28;
float kraft_y_min = -0.3;
float kraft_y_max = 1.28;
float kraft_z_min = -0.43;
float kraft_z_max = 1.5;

void movekraft(KDL::JntArray states_joints){

	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectorypoint;
	sensor_msgs::JointState jointstate;

	trajectory.joint_names.push_back("SA");
	trajectory.joint_names.push_back("SE");
	trajectory.joint_names.push_back("linkage_tr");
	trajectory.joint_names.push_back("WP");
	trajectory.joint_names.push_back("WY");
	trajectory.joint_names.push_back("WR");

	jointstate.name.push_back("SA");
	jointstate.name.push_back("SE");
	jointstate.name.push_back("linkage_tr");
	jointstate.name.push_back("WP");
	jointstate.name.push_back("WY");
	jointstate.name.push_back("WR");

	for (int i = 0; i < 6; i++){
		trajectorypoint.positions.push_back((float)states_joints.data(i,0));
		jointstate.position.push_back((float)states_joints.data(i,0));
		printf("%d %f\n", i, (float)states_joints.data(i,0));
		trajectorypoint.velocities.push_back(0.01);
		jointstate.velocity.push_back(1.0);
	}
	printf(" \n");
	trajectorypoint.time_from_start = ros::Duration(0.5);
	trajectory.points.push_back(trajectorypoint);

	pose_pub.publish(trajectory);
	real_pose_pub.publish(jointstate);
}

void checkSolution(geometry_msgs::Pose target_pose){
	float tolxyzrange = 0.01;
	float tolRxyzrange = 0.1;//0.4;
	KDL::Frame desired_end_effector_pose;
	tf::PoseMsgToKDL(target_pose, desired_end_effector_pose);
	KDL::JntArray joint_seed(chain.getNrOfJoints());
	KDL::JntArray joints_states(chain.getNrOfJoints());
	
	/*
	Bucle en el que se van aumentando las tolerancias de la orientación en caso de 
	no encuentrar solución para la ubicación indicada.
	*/
	for (int i = 0; i < 3; i++){
		tries++;
		float tolRxyz = tolRxyzrange*i;
		float tolxyz = tolxyzrange;
		KDL::Twist tolerances(	KDL::Vector(tolxyz, tolxyz, tolxyz),		// Tolerance x,y,z in meters
    							KDL::Vector(tolRxyz, tolRxyz, tolRxyz));	// Tolerance Rx,Rz,Rz in rad
		ros::Time t0 = ros::Time::now();
		int rc = ik_solver->CartToJnt(joint_seed, desired_end_effector_pose, joints_states, tolerances);
		ros::Time t1 = ros::Time::now();
		ros::Duration diff = t1-t0;
		
		if(rc == 1){
			tries_ok++;
			int msec = diff.nsec/1000000;
			printf("TIME %ld nsec %ld msec\n", diff.nsec, msec);
			printf("iter %d tol %f\n", i, tolRxyz);
			if (msec != 0){
				sum += msec;
				cnt++;
			}
			/*
			for (int i = 0; i < joints_states.data.size(); i++){
				printf("joint_%d: %f\n", i, (float)joints_states.data(i,0));
			}
			*/
			movekraft(joints_states);
			break;
    	}
	}
}

float convert_coords(float orig_coord, char axis){
	float converted_coord, phantom_min, phantom_max, kraft_min, kraft_max;
	switch (axis){
	case 'x':
		phantom_min = phantom_x_min;
		phantom_max = phantom_x_max;
		kraft_min = kraft_x_min;
		kraft_max = kraft_x_max;
		break;
	case 'y':
		phantom_min = phantom_y_min;
		phantom_max = phantom_y_max;
		kraft_min = kraft_y_min;
		kraft_max = kraft_y_max;
		break;
	case 'z':
		phantom_min = phantom_z_min;
		phantom_max = phantom_z_max;
		kraft_min = kraft_z_min;
		kraft_max = kraft_z_max;
		break;
	}

	if (orig_coord < 0){
		converted_coord = (orig_coord/phantom_min)*kraft_min;
	}else{
		converted_coord = (orig_coord/phantom_max)*kraft_max;
	}

	return converted_coord;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	gazebo_msgs::ModelState gizmopos;
	geometry_msgs::Pose target_pose;

    tf::StampedTransform transform, auxtransform;
    tf::TransformListener listener;
    //listener.waitForTransform("/phantom/base", "/phantom/stylus", ros::Time::now(), ros::Duration(0.25));
    listener.waitForTransform("/phantom/base", "/phantom/tip", ros::Time::now(), ros::Duration(0.25));
    try {
      //listener.lookupTransform("/phantom/base", "/phantom/stylus", ros::Time(0), transform);
      listener.lookupTransform("/phantom/base", "/phantom/tip", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::TransformStamped tfmsg;
    tf::transformStampedTFToMsg(transform, tfmsg);

    target_pose.position.x = convert_coords(tfmsg.transform.translation.x, 'x');
    target_pose.position.y = convert_coords(tfmsg.transform.translation.y, 'y');
    target_pose.position.z = convert_coords(tfmsg.transform.translation.z, 'z');
    target_pose.orientation.x = tfmsg.transform.rotation.x;
    target_pose.orientation.y = tfmsg.transform.rotation.y;
    target_pose.orientation.z = tfmsg.transform.rotation.z;
    target_pose.orientation.w = tfmsg.transform.rotation.w;

    gizmopos.model_name = "test_axis";
    gizmopos.pose.position.x = convert_coords(tfmsg.transform.translation.x, 'x');
    gizmopos.pose.position.y = convert_coords(tfmsg.transform.translation.y, 'y');
    gizmopos.pose.position.z = convert_coords(tfmsg.transform.translation.z, 'z');
    gizmopos.pose.orientation.x = tfmsg.transform.rotation.x;
    gizmopos.pose.orientation.y = tfmsg.transform.rotation.y;
    gizmopos.pose.orientation.z = tfmsg.transform.rotation.z;
    gizmopos.pose.orientation.w = tfmsg.transform.rotation.w;

    double roll, pitch, yaw;
    
    tf::Quaternion q(tfmsg.transform.rotation.x, 
        tfmsg.transform.rotation.y, 
        tfmsg.transform.rotation.z, 
        tfmsg.transform.rotation.w);
    
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //yaw += M_PI;
    //pitch += M_PI;
    roll -= M_PI/2;

    tf::Quaternion q2 = tf::createQuaternionFromRPY(roll, pitch, yaw);

    target_pose.orientation.x = q2[0];
    target_pose.orientation.y = q2[1];
    target_pose.orientation.z = q2[2];
    target_pose.orientation.w = q2[3];

    gizmopos.pose.orientation.x = q2[0];
    gizmopos.pose.orientation.y = q2[1];
    gizmopos.pose.orientation.z = q2[2];
    gizmopos.pose.orientation.w = q2[3];

    gizmo_pub.publish(gizmopos);

	checkSolution(target_pose);

	//KDL::JntArray joint_seed(chain.getNrOfJoints());
	//KDL::JntArray joints_states(chain.getNrOfJoints());
	//KDL::Twist tolerances(	KDL::Vector(0.01,0.01,0.01),	// Tolerance x,y,z in meters
    //						KDL::Vector(0.1,0.1,0.1));		// Tolerance Rx,Rz,Rz in rad

	//KDL::Frame desired_end_effector_pose;
	//tf::PoseMsgToKDL(target_pose, desired_end_effector_pose);
	//int rc = ik_solver->CartToJnt(joint_seed, desired_end_effector_pose, joints_states, tolerances);
    
    /*
    geometry_msgs::Pose robot_pose;
	tf::PoseKDLToMsg(desired_end_effector_pose, robot_pose);
	printf("pose x %f\n", tfmsg.transform.translation.x);
    printf("pose y %f\n", tfmsg.transform.translation.y);
    printf("pose z %f\n", tfmsg.transform.translation.z);
    printf("robot x %f\n", robot_pose.position.x);
    printf("robot y %f\n", robot_pose.position.y);
    printf("robot z %f\n", robot_pose.position.z);
    printf("axis x %f\n", gizmopos.pose.position.x);
    printf("axis y %f\n", gizmopos.pose.position.y);
    printf("axis z %f\n", gizmopos.pose.position.z);
    

    printf("rc %d\n", rc);

    if(rc == 1){
		movekraft(joints_states);
    }
    */
    
}

bool initIK(std::string chain_start, std::string chain_end, std::string urdf_param){
    double timeout = 0.005;
    double eps = 1e-5;
    bool valid;

    ik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps);
    valid = ik_solver->getKDLChain(chain);
	if (!valid){
		ROS_ERROR("There was no valid KDL chain found");
	}else{
		ROS_INFO("Valid KDL chain found !!");
	}
	return valid;
}

int main(int argc, char **argv){
	ros::init(argc,argv, "kraft_planner_tracik");
	ros::NodeHandle n, nh_gizmo, nh_pose, nh_realpose;

	sum = 0;
	cnt = 0;
	tries = 0;
	tries_ok = 0;

	ros::Subscriber p_sub = n.subscribe("/phantom/pose", 100, poseCallback);
	pose_pub = nh_pose.advertise<trajectory_msgs::JointTrajectory>("/joints_controller/command", 1000);
	real_pose_pub = nh_pose.advertise<sensor_msgs::JointState>("/ikjoints", 1000);
	gizmo_pub = nh_gizmo.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1000);

	if (!initIK("base_link", "end_effector", "/robot_description")){
		exit(1);
	}

	ros::spin();
	printf("Media: %f\n", sum/cnt);
	float percent = ((float)tries_ok/(float)tries)*100.0;
	printf("tries OK %d\n", tries_ok);
	printf("tries %d\n", tries);
	printf("Porcentaje: %f\n", percent);
	return 0;
}