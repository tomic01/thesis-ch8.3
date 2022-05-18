#include "ros/ros.h"
#include "monarch_msgs/BehaviorSelectionArray.h"
#include "monarch_msgs/BehaviorSelection.h"
#include "monarch_behaviors/BehaviorRequest.h"
 
#include <iostream>
#include <string.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"fake_SAP_output", ros::init_options::AnonymousName);
	ros::NodeHandle n;

	ros::ServiceClient SAP_output_client = n.serviceClient<monarch_behaviors::BehaviorRequest>("/mcentral/request_behavior");
	monarch_behaviors::BehaviorRequest behavior_request;

	//ros::Rate loop_rate(1);
	int instance = 0;

	while (ros::ok())
	{
		int i,j,input,numRobots;

		monarch_msgs::BehaviorSelection behavior;

		ROS_INFO("FSO: Enter behavior name:");
		std::cin >> behavior.name;

		behavior.instance_id = 0;
		//behavior.instance_id = instance;
		instance++;

		ROS_INFO("FSO: Enter number of robots:");
		numRobots = 0;
		std::cin >> numRobots;
		for(j = 1; j <= numRobots; j++)
		{
			ROS_INFO("FSO: Enter robot number:");
			std::cin >> input;
			behavior.robots.push_back(input);
		}
		//Adding the extra robot '0' (mpc01) if we activate interactivegame behavior
		if (behavior.name.compare("interactivegame") == 0)
		{
			behavior.robots.push_back(0);
		}

		monarch_msgs::KeyValuePair parameter_kvp;
		std::string gotolocation ("gotolocation");
		if (behavior.name.compare(gotolocation) == 0)
		{
			parameter_kvp.key = "location";
			ROS_INFO("Enter the desired location (e.g.: room1, room2 [defined in monarch_behaviors/behavior_executors/goto.py]):");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);
		}

		std::string distributed ("distributed");
		 if (behavior.name.compare(distributed) == 0)
		 {
		 	ROS_INFO("Robot ids sent to behavior for Voronoi division.");
		 	std::string ids [] = {"id0", "id1", "id2", "id3", "id4", "id5", "id6", "id7", "id8", "id9"};
		  	for (int n = 0; n < numRobots; n++) {
		  		parameter_kvp.key = ids[n];
		  		int temp = behavior.robots.at(n);
				char buffer [50];
				std::sprintf(buffer, "%d", temp);
				std::string apa (buffer);
				parameter_kvp.value = apa;
		  		behavior.parameters.push_back(parameter_kvp);
		  	}
		  }

		std::string approachperson ("approachperson");
		if (behavior.name.compare(approachperson) == 0)
		{
			parameter_kvp.key = "user_id";
			ROS_INFO("Enter the desired user_id (test 1001)");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);
		}

		std::string dispatchca ("dispatchca");
		if (behavior.name.compare(dispatchca) == 0)
		{
			parameter_kvp.key = "activated_cas";
			ROS_INFO("Enter the desired ca (next just parameters for c04):");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			std::string ca04("ca04");
			if (parameter_kvp.value.compare(ca04) == 0){
				parameter_kvp.key = "tell_user";
				ROS_INFO("Enter the value to tell_user parameter:");
				std::cin >> parameter_kvp.value;
				behavior.parameters.push_back(parameter_kvp);
			}

			std::string ca01("ca01");
			if (parameter_kvp.value.compare(ca01) == 0){
				parameter_kvp.key = "question_to_user";
				ROS_INFO("Enter the question_to_user parameter:");
				std::cin >> parameter_kvp.value;
				behavior.parameters.push_back(parameter_kvp);
			}

			behavior.resources.push_back("RVocSnd");
		}

		std::string cooperativepatrolling ("cooperativepatrolling");
		if (behavior.name.compare(cooperativepatrolling) == 0)
		{
			parameter_kvp.key = "bias_length";
			ROS_INFO("Enter the bias_length");
			std::cin >> parameter_kvp.value;
			//parameter_kvp.value = "1.5";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "formation_id";
			ROS_INFO("Enter the formation_id");
			std::cin >> parameter_kvp.value;
			//parameter_kvp.value = "4";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "max_cc";
			//ROS_INFO("Enter the max_cc");
			//std::cin >> parameter_kvp.value;
			parameter_kvp.value = "100000";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "leader_id";
			ROS_INFO("Enter the leader_id");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "follower_1";
			ROS_INFO("Enter the follower_1");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "follower_2";
			ROS_INFO("Enter the follower_2");
			std::cin >> parameter_kvp.value;
			// parameter_kvp.value = "0";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "follower_3";
			ROS_INFO("Enter the follower_3");
			std::cin >> parameter_kvp.value;
			//parameter_kvp.value = "0";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "number_of_robots";
			ROS_INFO("Enter the number_of_robots");
			std::cin >> parameter_kvp.value;
			//parameter_kvp.value = "2";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "switch_enabled";
			ROS_INFO("Enter the switch_enabled");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);
		}

		std::string interaction ("interaction");
		if (behavior.name.compare(interaction) == 0)
		{
			parameter_kvp.key = "communicative_act";
			//ROS_INFO("Enter the desired user_id (test 1001)");
			//std::cin >> parameter_kvp.value;
			parameter_kvp.value = "give_greetings";
			behavior.parameters.push_back(parameter_kvp);
		}
		
		std::string interactivegame ("interactivegame");
		if (behavior.name.compare(interactivegame) == 0)
		{
			parameter_kvp.key = "players";
			ROS_INFO("Enter the name of the players");

			// Magic: Flush the buffer otherwise i can t read strings with spaces
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			char field[100];
			std::cin.getline(field, 100, '\n');
			parameter_kvp.value = field;
			//parameter_kvp.value = "mbotXX human1";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "rows";
			ROS_INFO("Enter the number of rows");
			std::cin >> parameter_kvp.value;
			//parameter_kvp.value = "4";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "cols";
			ROS_INFO("Enter the number of cols");
			std::cin >> parameter_kvp.value;
			// parameter_kvp.value = "4";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "level";
			ROS_INFO("Enter the level");
			std::cin >> parameter_kvp.value;
			// parameter_kvp.value = "0";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "tracking";
			ROS_INFO("Enter if activate tracking");
			std::cin >> parameter_kvp.value;
			// parameter_kvp.value = "0";
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "interaction";
			ROS_INFO("Enter type of desired interaction (none, nonverbal, verbal):");
			std::cin >> parameter_kvp.value;
			// parameter_kvp.value = "0";
			behavior.parameters.push_back(parameter_kvp);

			//resources
			behavior.resources.push_back("RNAV");

		}

		//resources
		std::string moving ("moving");
		if (behavior.name.compare(moving) != 0)
		{
			behavior.resources.push_back("RNAV");
		}

		std::string dock ("dock");
		if (behavior.name.compare(dock) == 0)
		{
			behavior.resources.push_back("RLED");
		}

		if (behavior.name.compare(moving) == 0)
		{
			behavior.resources.push_back("RLED");
			behavior.resources.push_back("RVocSnd");
		}

		std::string idle ("idle");
		if (behavior.name.compare(idle) == 0)
		{
			behavior.resources.push_back("RLED");
			behavior.resources.push_back("RVocSnd");
		}


		std::string following ("following");
		if (behavior.name.compare(following) == 0)
		{
			parameter_kvp.key = "user_id";
			ROS_INFO("Enter the desired user_id (test 1001)");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);
		}


		std::string escorting ("escorting");
		if (behavior.name.compare(escorting) == 0)
		{
			parameter_kvp.key = "user_id";
			ROS_INFO("Enter the desired user_id (test 1001)");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "x";
			ROS_INFO("Enter the desired x position for the escorting goal on the map ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "y";
			ROS_INFO("Enter the desired y position for the escorting goal on the map ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

		}

		std::string getrfid ("getrfid");
		if (behavior.name.compare(getrfid) == 0)
		{
			parameter_kvp.key = "camera_id";
			ROS_INFO("Enter the desired camera_id (test 1001)");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);
		}

		std::string catchandtouch ("catchandtouch");
		if (behavior.name.compare(catchandtouch) == 0)
		{
			parameter_kvp.key = "camera_id";
			ROS_INFO("Enter the desired camera_id (test 1001)");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "x";
			ROS_INFO("Enter the desired x position for the catchandtouch goal on the map ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "y";
			ROS_INFO("Enter the desired y position for the catchandtouch goal on the map ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "theta";
			ROS_INFO("Enter the desired angle position for the catchandtouch goal on the map ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "vel";
			ROS_INFO("Enter the desired maximum speed for the robot ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);
		}

		std::string catchandtouchback ("catchandtouchback");
		if (behavior.name.compare(catchandtouchback) == 0)
		{
			parameter_kvp.key = "camera_id";
			ROS_INFO("Enter the desired camera_id (test 1001)");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "vel";
			ROS_INFO("Enter the desired maximum speed for the robot ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);
		}

		std::string lead ("lead");
		if (behavior.name.compare(lead) == 0)
		{
			parameter_kvp.key = "camera_id";
			ROS_INFO("Enter the desired camera_id (test 1001)");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "x";
			ROS_INFO("Enter the desired x position for the lead goal on the map ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "y";
			ROS_INFO("Enter the desired y position for the lead goal on the map ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "theta";
			ROS_INFO("Enter the desired angle position for the lead goal on the map ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);

			parameter_kvp.key = "rfid_tag";
			ROS_INFO("Enter the person rfid tag ");
			std::cin >> parameter_kvp.value;
			behavior.parameters.push_back(parameter_kvp);
		}


		ROS_INFO("FSO: Execute behavior? 1; Cancel behavior? 0:");
		std::cin >> input;
		behavior.active = input;

		behavior_request.request.behavior = behavior;

		SAP_output_client.call(behavior_request);

		if (behavior_request.response.success)
			ROS_INFO("FSO: behavior command sent from planner to Global Behavior Manager successfully");
		else
			ROS_INFO("FSO: behavior command sent from planner to Global Behavior Manager unsuccessfully");

		ros::spinOnce();

		//loop_rate.sleep();
	}

	return 0;
}
