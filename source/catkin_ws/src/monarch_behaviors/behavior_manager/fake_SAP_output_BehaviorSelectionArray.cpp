#include "ros/ros.h"
#include "monarch_msgs/BehaviorSelectionArray.h"
#include "monarch_msgs/BehaviorSelection.h"

#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"fake_SAP_output");
	ros::NodeHandle n;

	ros::Publisher SAP_output_pub = n.advertise<monarch_msgs::BehaviorSelectionArray>("SAP_behavior_output",1);

	//ros::Rate loop_rate(1);

	while (ros::ok())
	{
		int i,j,input,numBehaviors,numRobots;

		monarch_msgs::BehaviorSelectionArray msg;

		ROS_INFO("FSO: Enter number of behaviors:");
		std::cin >> numBehaviors;
		for(i = 1; i <= numBehaviors; i++)
		{
			monarch_msgs::BehaviorSelection behavior;
	
			ROS_INFO("FSO: Enter behavior number:");
			std::cin >> input;
			behavior.id = input;
	
			ROS_INFO("FSO: Enter number of robots:");
			numRobots = 0;
			std::cin >> numRobots;
			for(j = 1; j <= numRobots; j++)
			{
				ROS_INFO("FSO: Enter robot number:");
				std::cin >> input;
				behavior.robots.push_back(input);
			}
	
			monarch_msgs::KeyValuePair parameter_kvp;
			parameter_kvp.key = "location";
			parameter_kvp.value = "play-room";
			behavior.parameters.push_back(parameter_kvp);	
	
			behavior.start_earliest = ros::Time::now();
			behavior.start_latest = ros::Time::now();
			behavior.end_earliest = ros::Time(0);
			behavior.end_latest = ros::Time(0);

			msg.array.push_back(behavior);
		}	

		SAP_output_pub.publish(msg);	

		ROS_INFO("FSO: %d behavior commands sent from planner to Global Behavior Manager",numBehaviors);	

		ros::spinOnce();

		//loop_rate.sleep();
	}

	return 0;
}