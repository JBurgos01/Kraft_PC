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
#include <fstream>
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

#include <ros/package.h>

ros::Publisher SA_pub, SE_pub, tr_pub, WP_pub, WY_pub, WR_pub, pose_pub, gizmo_pub;
TRAC_IK::TRAC_IK *ik_solver;
KDL::Chain chain;
std::ifstream infile;
std::ofstream outfile;
std::ofstream ikpointsfile;

double h_matrix[4][4];

int globalcnt = 0;

void movekraft(KDL::JntArray states_joints){
	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectorypoint;

	trajectory.joint_names.push_back("SA");
	trajectory.joint_names.push_back("SE");
	trajectory.joint_names.push_back("linkage_tr");
	trajectory.joint_names.push_back("WP");
	trajectory.joint_names.push_back("WY");
	trajectory.joint_names.push_back("WR");

	for (int i = 0; i < 6; i++){
		trajectorypoint.positions.push_back((float)states_joints.data(i,0));
		trajectorypoint.velocities.push_back(1.0);
	}
	trajectorypoint.time_from_start = ros::Duration(1, 0);
	trajectory.points.push_back(trajectorypoint);

	pose_pub.publish(trajectory);
}

//void testPoint(const geometry_msgs::PoseStamped::ConstPtr& msg) {
void testPoint() {

	gazebo_msgs::ModelState gizmopos;

	// Tolerances
	double tolxyz = 0.1;
	double tolRxyz = 0.1;

	double qx, qy, qz, qw;

	double Xx = h_matrix[0][0];
	double Yx = h_matrix[1][0];
	double Zx = h_matrix[2][0];
	double Xy = h_matrix[0][1];
	double Yy = h_matrix[1][1];
	double Zy = h_matrix[2][1];
	double Xz = h_matrix[0][2];
	double Yz = h_matrix[1][2];
	double Zz = h_matrix[2][2];
	double x = h_matrix[0][3];
	double y = h_matrix[1][3];
	double z = h_matrix[2][3];

	KDL::Rotation(Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz).GetQuaternion(qx, qy, qz, qw);
	gizmopos.model_name = "test_axis";
    gizmopos.pose.position.x = x;
    gizmopos.pose.position.y = y;
    gizmopos.pose.position.z = z;
    gizmopos.pose.orientation.x = 0.0;
    gizmopos.pose.orientation.y = 0.0;
    gizmopos.pose.orientation.z = 0.0;
    gizmopos.pose.orientation.w = 1.0;

    gizmo_pub.publish(gizmopos);
	
	KDL::Twist tolerances(	KDL::Vector(tolxyz, tolxyz, tolxyz),		// Tolerance x,y,z in meters
    						KDL::Vector(tolRxyz, tolRxyz, tolRxyz));	// Tolerance Rx,Rz,Rz in rad

	//KDL::Frame desired_end_effector_pose(KDL::Rotation(Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz), KDL::Vector(x, y, z));
	double roll, pitch, yaw;
	KDL::Rotation(Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz).GetRPY(roll, pitch, yaw);
	//printf("RPY %f %f %f\n", roll, pitch, yaw);
	//printf("Q %f %f %f %f\n", qx, qy, qz, qw);
	KDL::Frame desired_end_effector_pose(KDL::Rotation::RPY(roll, pitch, yaw), KDL::Vector(x, y, z));
	KDL::JntArray joint_seed(chain.getNrOfJoints());
	KDL::JntArray joints_states(chain.getNrOfJoints());

	int rc = ik_solver->CartToJnt(joint_seed, desired_end_effector_pose, joints_states, tolerances);
/*
	for (int i = 0; i <= 3; i++){
    	for (int j = 0; j <= 3; j++){
    		printf("%f ", desired_end_effector_pose(i,j));
    	}
    	printf("\n");
    }
*/
	printf("%d rc %d\n", globalcnt, rc);
	int c;
	if (rc == 1){
		for (int r = 0; r < 4; r++){
			for (c = 0; c < 4; c++){
				if (r == 0 && c == 0){
					ikpointsfile << ((globalcnt+1)*4)-3 << ",";
				}else{
					ikpointsfile << " " << ",";
				}
				ikpointsfile << h_matrix[r][c] << ",";
			}
			if (r == 0){
				for (int s = 0; s < 6; s++){
					ikpointsfile << joints_states.data(s,0) << ",";
				}
			}
			ikpointsfile << "\n";
		}
		outfile << rc << ",";
		for (int s = 0; s < 6; s++){
			outfile << joints_states.data(s,0) << ",";
		}
		outfile << "\n\n\n\n";
	}else{
		outfile << rc << "\n\n\n\n";
	}
	globalcnt++;
	/*
	printf("joints_states.data.size() %d\n", joints_states.data.size());
	if(rc == 1){
		for (int i = 0; i < joints_states.data.size(); i++){
			printf("joint_%d: %f\n", i, (float)joints_states.data(i,0));
		}
		movekraft(joints_states);
	}  
	*/
}

bool openInfile(){
	bool statusopen;
	infile.open ("/home/david/catkin_ws/src/kraft/kraft_planner/src/points/v2/mth.csv");
	if (infile.is_open()){
		std::cout << "IN File open OK\n";
		statusopen = true;
	}else{
		std::cout << "IN File open NO OK, not reading...\n";
		statusopen = false;
	}
	return statusopen;
}

bool openOutfile(){
	bool statusopen;
	outfile.open ("/home/david/catkin_ws/src/kraft/kraft_planner/src/points/v2/out.csv");
	if (outfile.is_open()){
		std::cout << "OUT File open OK\n";
		statusopen = true;
	}else{
		std::cout << "OUT File open NO OK, not reading...\n";
		statusopen = false;
	}
	return statusopen;
}

bool openIKfile(){
	bool statusopen;
	ikpointsfile.open ("/home/david/catkin_ws/src/kraft/kraft_planner/src/points/v2/ik.csv");
	if (ikpointsfile.is_open()){
		std::cout << "IK File open OK\n";
		statusopen = true;
	}else{
		std::cout << "IK File open NO OK, not reading...\n";
		statusopen = false;
	}
	return statusopen;	
}

void readFile(){
	std::string line;
	std::string s;
	if (infile.good()){
		for (int i = 0; i < 4; i++){
			getline(infile, line);
			std::istringstream iss(line);
			for (int j = 0; j < 4; j++){
				getline(iss, s, ',');
				h_matrix[i][j] = std::stod(s.c_str(), NULL);
				//printf("(%d %d) - %f\n", i, j, std::stod(s.c_str(), NULL));
			}
			std::cout << i << " " << std::string(line, 0, line.length()) << "\n";
		}
	}
	/*
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			printf("(%d %d) - %f\n", i, j, h_matrix[i][j]);
		}
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
	ros::init(argc,argv, "test_points");
	ros::NodeHandle n, nh_gizmo, nh_pose;

	//ros::Subscriber p_sub = n.subscribe("/phantom/pose", 100, testPoint);
	pose_pub = nh_pose.advertise<trajectory_msgs::JointTrajectory>("/joints_controller/command", 1000);
	gizmo_pub = nh_gizmo.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1000);

	if (!initIK("base_link", "end_effector", "/robot_description")){
		exit(1);
	}
	
	if (openInfile() && openOutfile() && openIKfile()){
		while(globalcnt < 5333){//ros::ok()){
		//for (int c = 0; c < 36; c++){
			readFile();
			testPoint();
		}
		infile.close();
		outfile.close();
		ikpointsfile.close();
	}
	//ros::spin();
	return 0;
}