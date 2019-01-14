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
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <ros/serialization.h>

#define SRV_IP "192.168.0.107"
#define PORT 5052
#define BUFLEN 1024
#define NPACK 10

using namespace std;

int main(int argc, char *argv[]){

 	struct sockaddr_in addr, src_addr;
 	int s;
	uint slen = sizeof(src_addr);

	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0){
		printf("ERROR opening socket\n");
        exit(1);
	}else{
    	printf("OK opening socket\n");
    }
	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0){
		printf("ERROR binding socket\n");
        exit(1);
	}else{
    	printf("OK binding socket\n");
    }

    memset(&src_addr, 0, sizeof(struct sockaddr_in));
    src_addr.sin_family = AF_INET;
    if (inet_aton(SRV_IP, &(src_addr.sin_addr)) == 0){
    	printf("Failed copying address\n");
    	exit(1);
    }else{
    	printf("OK copying address\n");
    }

    sensor_msgs::JointState jointbuffer;
	uint32_t serial_size = ros::serialization::serializationLength(jointbuffer);

	cout << "serial_size: " << serial_size << endl;

	boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

    while(1){
    	//char buffer[BUFLEN];
    	
    	//char* buffer = new char[BUFLEN];

		//int n = read(s, buffer, BUFLEN);
		//int n = read(s, jointbuffer, sizeof(*jointbuffer));

		//int n = read(s, fbuffer, 7);
		//int n = recv(s, buffer, BUFLEN, 0);


		//int n = recvfrom(s, (char*)&buffer, sizeof(buffer), 0, (struct sockaddr *)&src_addr, &slen);
		int n = recvfrom(s, (char*)buffer.get(), sizeof(buffer), 0, NULL, 0);
		//int n = recvfrom(s, jointbuffer, sizeof(*jointbuffer), 0, NULL, 0);
		// recvfrom(int sockfd, void *buf, size_t len, int flags, struct sockaddr *src_addr, socklen_t *addrlen);
	    
	    if (n < 0){
	        printf("ERROR reading from socket\n");
	        exit(0);
	    }else{

	    	ros::serialization::IStream stream(buffer.get(), serial_size);
			ros::serialization::deserialize(stream, jointbuffer);


	    	cout << "Mensaje recibido:" << endl << endl;

	    	printf("\t-seq: %d", jointbuffer.header.seq);



	    	// sensor_msgs::JointState message;
	    	// sensor_msgs::JointState::deserialize(message, buffer);
	    	
	    	//ros::serialization::deserializeMessage(buffer, &jointbuffer);
	    	//ros::serialization::Serializer<sensor_msgs::JointState>::read(buffer, jointbuffer);
	    	//printf("%f\n", jointbuffer->position[0]);
	    	/*
	    	// SA (53 41 02)
	    	printf("%c%c %x %x\n", buffer[24], buffer[25], buffer[24], buffer[25]);
	    	// SE
	    	printf("%c%c %x %x\n", buffer[30], buffer[31], buffer[30], buffer[31]);
	    	// EL
	    	printf("%c%c %x %x\n", buffer[36], buffer[37], buffer[36], buffer[37]);
	    	// WP
	    	printf("%c%c %x %x\n", buffer[42], buffer[43], buffer[42], buffer[43]);
	    	// WY
	    	printf("%c%c %x %x\n", buffer[48], buffer[49], buffer[48], buffer[49]);
	    	// WR
	    	printf("%c%c %x %x\n", buffer[54], buffer[55], buffer[54], buffer[55]);
	    	*/

	    	//uint32_t seq = *(reinterpret_cast<uint32_t*>(buffer));

	    	// char a0=buffer[0], a1=buffer[1], a2=buffer[2], a3=buffer[3];

	    	// uint32_t seq = (uint32_t)(a0<<24) + (uint32_t)(a1<<16) + (uint32_t)(a2<<8) + (uint32_t)(a3);

	    	// cout << "seq: " << seq << endl;

	    	// for(int i = 0; i < n; i++)
	    	// {
	    	// 	printf("%c, ", buffer[i]);
	    	// }

/*
	    	for (int i = 0; i < n; i++){
	    		for (int j = i*5; j < (i*5)+5; j++){
	    			printf("%d ", buffer[j]);
	    		}
	    		printf("\n");
	    	}


	    	for (int i = 24; i < 60; i++){
	    		//printf("%s ", buffer);
	    		printf("%c ", buffer[i]);
	    	}
	    	*/
	    	printf("\n");
	    }
	    printf("\n-----------------\n");
	    printf("\n\n");
	}

	close(s);
	return 0;
}