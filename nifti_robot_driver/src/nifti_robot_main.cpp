
#include <iostream>
#include <ros/ros.h>
#include "nifti_robot.h"


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "nifti_robot_node");

	// must be done after ROS init
	NiftiRobot robot;

	robot.run();

	return 0;

}
