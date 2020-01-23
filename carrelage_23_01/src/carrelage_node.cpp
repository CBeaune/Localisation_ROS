#include <ros/ros.h> 
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include "usefulFunctions.h"

using namespace std;

double x,y,theta;
Quaternion q;
int mesures;


void odomCallBack(geometry_msgs::Pose pose){
	x = pose.position.x;
	y = pose.position.y;

	q.x=pose.orientation.x;
	q.y=pose.orientation.y;
	q.z=pose.orientation.z;
	q.w=pose.orientation.w;
	theta = ToEulerAngles(q).yaw;
}


void sensorCallBack(std_msgs::Int8 sensor){
	mesures=sensor.data;
}


int main(int argc, char**argv){
	ros::init(argc,argv,"carrelage");
	ros::NodeHandle nh("~");
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("/odom/pose/pose",1,odomCallBack);
	ros::Subscriber sensor_sub = nh.subscribe<std_msgs::Int8>("/detect",1,sensorCallBack);
	
	while(ros::ok()){
		
		ros::spinOnce();
	
	
	}
}
