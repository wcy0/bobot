/*
 * main.cpp
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */
#include "ros/ros.h"
#include "serial_node.h"
#include "serial_port.h"

using namespace std;
using namespace BO_SerialNode;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "BO_SerialNode");

	SerialNode serialnode(ros::NodeHandle(), ros::NodeHandle("~"));
	
	ros::Rate rate(30);
	
	ros::AsyncSpinner spinner(1);
  	
	spinner.start();

	while(ros::ok())
  	{
	        serialnode.updateSpeed();
		rate.sleep();
	}
	spinner.stop();

	return 0;
}


