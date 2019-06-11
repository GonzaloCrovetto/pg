#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

#include <ros/ros.h>
#include "std_msgs/Char.h"

int kfd = 0;
struct termios cooked, raw;

int main(int argc, char** argv){

	ROS_INFO("pg: Keyboard for Lineal Explorer");
	ros::init(argc, argv, "pg_LEKeyboard");
	ros::NodeHandle node;
	ros::Rate loop_rate(20);

	ros::Publisher key_Publisher = node.advertise<std_msgs::Char>("LEKeyboard", 1);

	//Things needed to read the input
	char keyRead='X';
	std_msgs::Char key;
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	ROS_INFO("Waiting for Subscribers");
	while (ros::ok() and key_Publisher.getNumSubscribers()==0){
		loop_rate.sleep();
		ros::spinOnce();
	};
	ROS_INFO("Ready");
	ROS_INFO("Typical WASD movement. Q stops movement, Z to exit, and everything else to go back to autonomous movement");

	while(ros::ok() and  keyRead!='z' and keyRead!='Z')
	{
		// Get the next event from the keyboard
		if(read(kfd, &keyRead, 1) < 0){
			ROS_INFO("ENDING (I saw this: 0x%02X)\n", keyRead);
			exit(0);
		}

		switch(keyRead){
			case 's': case 'S':/*S*/
				key.data='S';
				ROS_INFO("BACK");
			break;
			case 0x77: case 0x57:/*W*/
				key.data='W';
				ROS_INFO("FORWARD");
			break;
			case 0x64: case 0x44:/*D*/
				key.data='D';
				ROS_INFO("TURNING RIGHT");
			break;
			case 0x61: case 0x41:/*A*/
				key.data='A';
				ROS_INFO("TURNING LEFT");
			break;
			case 0x71: case 0x51:/*q*/
				key.data='Q';
				ROS_INFO("STOP MOVEMENT");
			break;
			default:///FIXME , CHANGE FOR CASE 'X'
				key.data='X';
				ROS_INFO("BACK TO AUTONOMOUS MOVEMENT");
			break;
		}

		key_Publisher.publish(key);

		loop_rate.sleep();
		ros::spinOnce();
	}
	tcsetattr(kfd, TCSANOW, &cooked);
}
