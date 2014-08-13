#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h"
#include "se306_project1/DoctorMsg.h"

#include <sstream>
#include "math.h"
#include "Resident.h"	
	
void Resident::StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 5 + msg.pose.pose.position.x;
	py =10 + msg.pose.pose.position.y;
	//ROS_INFO("Current x position is: %f", px);
	//ROS_INFO("Current y position is: %f", py);
}

void Resident::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

//doctor will heal resident when they are next to each other
void Resident::doctor_callback(se306_project1::DoctorMsg msg)
{
	if (msg.healResident == true)
	 	health = 100;
}

int Resident::run(int argc, char **argv)
{
	//initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = M_PI/2.0;
	px = 10;
	py = 20;
	
	//Initial velocity
	linear_x = 0.2;
	angular_z = 0.2;

	health = 100;
	boredom = 100;
	hunger = 100;
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Resident");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//custom message/topic publisher "resident/state" for now
	//ros::Publisher Resident_pub = n.advertise<std_msgs::String>("residentStatus",1000); 
	ros::Publisher resident_pub = n.advertise<se306_project1::ResidentMsg>("residentStatus",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Resident::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Resident::StageLaser_callback, this);

	//subscribe to doctor messages
	ros::Subscriber doctor_sub = n.subscribe<se306_project1::DoctorMsg>("healResident",1000, &Resident::doctor_callback, this);

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	int hungerReductionRate = 1; //1 hunger point reduction per second
	int healthReductionRate = 1; // 0.1 health point reduction per second
	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
		
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		// Reduces hunger every second
		if (count % 10 == 0){
				hunger -= hungerReductionRate;
				health -= healthReductionRate;
		}
		
		
		//std_msgs::String msg;
		//std::stringstream ss;
		//ss << "Hello world" << hunger;
		//msg.data = ss.str();
		
		//custom resident message publisher
		
		se306_project1::ResidentMsg msg; 
		msg.health = health;
		msg.hunger = hunger;

		resident_pub.publish(msg);
		
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}

/* 
	Redirects to main function (run()) of the node.
*/
int main(int argc, char **argv) {
	Resident *a = new Resident();
	a->Resident::run(argc, argv);
}
