#include <ros/ros.h> 
#include <geometry_msgs/Position.h>
using namespace std;
double x,y,theta
Quaternion q;

void stateCallBack(geometry_msgs::Pose pose){
	x = pose.position.x;
	y = pose.position.y;

	q.x=pose.orientation.x;
	q.y=pose.orientation.y;
	q.z=pose.orientation.z;
	q.w=pose.orientation.w;
	theta = ToEulerAngles(q).yaw;
}

int main(int argc, char**argv){
	ros::init(argc,argv,"carrelage");
	ros::NodeHandle nh;
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("/odom/pose/pose",stateCallBack);
	ros::Subscriber sensor_sub = nh.subscribe<geometry_msgs::Pose>("/odom/pose/pose",stateCallBack);
	
	while(ros::ok()){
		
		ros::spinOnce();
	
	
	
	
	
	
	
	
	
	}
}
