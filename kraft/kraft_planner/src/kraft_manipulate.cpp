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

#define SRV_IP "192.168.0.4"
#define READPORT 5051
#define WRITEPORT 5052
#define BUFLEN 1024
#define NPACK 10

using namespace std;

char* createMessage(){
    //ros::SerializedMessage
/*
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
*/
    sensor_msgs::JointState message;
    //char *message;
    message.header.stamp = ros::Time::now();
    message.name.resize(6);
    message.name = {"SA","SE","EL","WP","WY","WR"};
    message.position.resize(6);
    message.position = {0.058, -1.649, -0.183, -0.112, 0.001, 2.745};
    //char * outbuffer;
    //message.serialize(outbuffer);
    //return ros::serialization::serializeMessage(message);
    char *m;
    return m;
}

int main(int argc, char *argv[]){

    struct sockaddr_in addr;
    int s;
    uint slen = sizeof(addr);

    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0){
        printf("ERROR opening socket\n");
        exit(1);
    }else{
        printf("OK opening socket\n");
    }
    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(WRITEPORT);

    if (inet_aton(SRV_IP, &(addr.sin_addr)) == 0){
        printf("Failed copying address\n");
        exit(1);
    }else{
        printf("OK copying address\n");
    }

    while(1){
        char *message = createMessage();
        if (sendto(s, message, sizeof(message), 0, (struct sockaddr *) &addr, slen) < 0){
            printf("Failed sending message\n");
            exit(1);
        }
    }

    close(s);
    return 0;
}