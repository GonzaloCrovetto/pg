#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/LaserScan.h>

#define tooClose 0.18
#define frontVel 10.0
#define turnVel 2.0
#define tooFar 0.4
#define angDiv 8 //To divide the angle of usefull data provided by the laser (usefull to try new settings without changing the robot)
#define turnAround 18 //To set a 1 en turnAround chance to turn around when deciding to turn

struct velMessage {
	std_msgs::Float64 v_left;
	std_msgs::Float64 v_back;
	std_msgs::Float64 v_right;
};

double minFrontDist=1000;
double maxLeftDist=-1;
double maxRightDist=-1;
bool newTurn=true,commandChange=false;
char command='X';
velMessage movement, turn, front, left, right, stop, back;


void setDistances(const sensor_msgs::LaserScan::ConstPtr &senMsg);
void autonomousMovement();
void getKey(const std_msgs::Char &keyMsg);


int main(int argc, char** argv){
	srand(time(NULL));

  ROS_INFO("pg: Lineal Explorer with open_base");
	ros::init(argc, argv, "pg_openBase_linealExplorer");
  ros::NodeHandle node;
	ros::Rate loop_rate(20);

  ros::Publisher v_left_Publisher;
	ros::Publisher v_back_Publisher;
	ros::Publisher v_right_Publisher;
	ros::Subscriber scan_Subscriber;
	ros::Subscriber key_Subscriber;
	v_left_Publisher= node.advertise<std_msgs::Float64>("/open_base/left_joint_velocity_controller/command", 1);
	v_back_Publisher = node.advertise<std_msgs::Float64>("/open_base/back_joint_velocity_controller/command", 1);
	v_right_Publisher = node.advertise<std_msgs::Float64>("/open_base/right_joint_velocity_controller/command", 1);
	scan_Subscriber = node.subscribe("/open_base/laser/scan",1, setDistances);
	key_Subscriber = node.subscribe("LEKeyboard", 1,getKey);

	front.v_left.data=-1*frontVel; 	front.v_back.data=0; 					front.v_right.data=1*frontVel;
	back.v_left.data=1*frontVel; 		back.v_back.data=0; 					back.v_right.data=-1*frontVel;
	left.v_left.data=1*turnVel; 		left.v_back.data=1*turnVel; 	left.v_right.data=1*turnVel;
	right.v_left.data=-1*turnVel; 	right.v_back.data=-1*turnVel; right.v_right.data=-1*turnVel;
	stop.v_left.data=0; 						stop.v_back.data=0; 					stop.v_right.data=0;

	ROS_INFO("Waiting for the robot");
	while (ros::ok() and v_left_Publisher.getNumSubscribers()==0){
		loop_rate.sleep();
		ros::spinOnce();
	};
	ROS_INFO("Ready");

	while (ros::ok()){

		if(command=='X')
			autonomousMovement();
		else if(commandChange){//There is no need to do this if the command has not changed
			switch(command){
				case 'A': movement=left; break; //Turn left
				case 'D': movement=right; break; //Turn right
				case 'S': movement=back; break; //Turn right
				case 'W': movement=front; break; //Move forward
				/*case 'Q':*/ default: movement=stop; break;//Dont move
			}
			newTurn=true;
			commandChange=false;
		}
		// We repeat the message in case the first one goes missing
		v_left_Publisher.publish(movement.v_left);
		v_back_Publisher.publish(movement.v_back);
		v_right_Publisher.publish(movement.v_right);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}

///Commands from the user in order to be able to move the robot with the keyboard
void getKey(const std_msgs::Char &keyMsg) {
	char prevComm=command;
	command=keyMsg.data;
	if(prevComm!=command)
		commandChange=true;
}

///Decides where to move depending on the distances values
void autonomousMovement(){
	if(minFrontDist<tooClose){//Time to turn
		if(newTurn){//In order to keep turning to the same side untill we start going straight
			if(maxRightDist>tooFar and maxLeftDist>tooFar){
				if (rand()%2==0)
					turn=left;
				else
					turn=right;
				ROS_INFO("RANDOM TURN> FrontDist:%.2f LeftDist:%.2f, RightDist:%.2f",minFrontDist, maxLeftDist, maxRightDist);

			}
			else if(maxRightDist>maxLeftDist)
				if (rand()%turnAround==0){
					turn=left;
					ROS_INFO("TURN ARROUND> FrontDist:%.2f LeftDist:%.2f, RightDist:%.2f",minFrontDist, maxLeftDist, maxRightDist);
				}
				else{
					turn=right;
					ROS_INFO("NORMAL TURN> FrontDist:%.2f LeftDist:%.2f, RightDist:%.2f",minFrontDist, maxLeftDist, maxRightDist);
				}
			else
				if (rand()%turnAround==0){
					turn=right;
					ROS_INFO("TURN ARROUND> FrontDist:%.2f LeftDist:%.2f, RightDist:%.2f",minFrontDist, maxLeftDist, maxRightDist);
				}
				else{
					turn=left;
					ROS_INFO("NORMAL TURN> FrontDist:%.2f LeftDist:%.2f, RightDist:%.2f",minFrontDist, maxLeftDist, maxRightDist);
				}
			newTurn=false;
		}
		movement=turn;
	}
	else{
		if(!newTurn){
			ROS_INFO("FORWARD> FrontDist:%.2f LeftDist:%.2f, RightDist:%.2f",minFrontDist, maxLeftDist, maxRightDist);
			newTurn=true;
		}
		movement=front;
	}
}

///Sets the min distance to the front (using the front third of the angle of readings), and also the max distance towards each side (starting after said third)
void setDistances(const sensor_msgs::LaserScan::ConstPtr &senMsg) {
  int last, i=0;
	double leftMax=-1, frontMin=1000, rightMax=-1;
	// Only the middle third of the readings to calculate the min front distance
	// From there it depends on the andDiv variable to calculate the readings angle for each side
	last= senMsg->ranges.size();
	for(i=last*(angDiv-1)/angDiv;i>=last/angDiv;i--){//Change this if you no longer want to divide the angle from the sides
		if( (senMsg->ranges[i] >= senMsg->range_min) and (senMsg->ranges[i] <= senMsg->range_max))
				if(i<=last*2/3 and i>=last/3 and senMsg->ranges[i] < frontMin)
					frontMin=senMsg->ranges[i];
				if(i>=last*2/3  and senMsg->ranges[i] > leftMax)
					leftMax=senMsg->ranges[i];
				if(i<=last/3 and senMsg->ranges[i] > rightMax)
					rightMax=senMsg->ranges[i];
	}
	if (frontMin<1000)
			minFrontDist=frontMin;
	if (leftMax>=0)
		 maxLeftDist=leftMax;
	if (rightMax>=0)
		 maxRightDist=rightMax;
}
