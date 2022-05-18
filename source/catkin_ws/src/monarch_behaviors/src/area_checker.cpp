#include <math.h> 

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <monarch_behaviors/CheckAreaClearAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#define BETWEEN(value, min, max) (value < max && value > min)

class LaserScanToPointCloud
{
protected:

	ros::NodeHandle n_;
	laser_geometry::LaserProjection projector_;
	tf::TransformListener listener_;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
	tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
	ros::Publisher scan_pub_;

public:

	LaserScanToPointCloud(ros::NodeHandle n): 
		n_(n),
		laser_sub_(n_, "scan", 10),
		laser_notifier_(laser_sub_,listener_, "/map", 10)
	{
		laser_sub_.unsubscribe();
		laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
		laser_notifier_.setTolerance(ros::Duration(0.1));
		scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("scan_cloud",1);
	}

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		sensor_msgs::PointCloud cloud;
		try
		{
			projector_.transformLaserScanToPointCloud("/map",*scan_in, cloud,listener_);
		}
		catch (tf::TransformException& e)
		{
			std::cout << e.what();
			return;
		}

		scan_pub_.publish(cloud);
	}

	void unsubscribe_from_scan()
	{
		laser_sub_.unsubscribe();
	}

	void subscribe_to_scan()
	{
		laser_sub_.subscribe();
	}
};

class CheckAreaClearAction
{
protected:

	ros::NodeHandle nh_;
	std::string action_name_;
	actionlib::SimpleActionServer<monarch_behaviors::CheckAreaClearAction> as_; 
	monarch_behaviors::CheckAreaClearResult result_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_nav_;
	LaserScanToPointCloud lstpc_;
	ros::Subscriber pt_cloud_sub_;
	geometry_msgs::Pose initial_pose_;
	std::vector<tf::Vector3> area_corners_;
	unsigned long int point_count_;

public:

	CheckAreaClearAction(std::string name) :
		action_name_(name),
		as_(nh_, name, boost::bind(&CheckAreaClearAction::executeCB, this, _1), false),
		ac_nav_("move_base", true),
		lstpc_(nh_)
		{
			as_.start();
			ac_nav_.waitForServer();
			pt_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud>("scan_cloud", 10, boost::bind(&CheckAreaClearAction::ptCloudCallback, this, _1));

			point_count_ = 0;
		}

	~CheckAreaClearAction(void)
	{
	}

	void ptCloudCallback(const sensor_msgs::PointCloud::ConstPtr& ptcloud_in)
	{
		for (unsigned int i = 0; i < ptcloud_in->points.size(); ++i)
		{
			// a,b,c are corners of the rectangular area (with ab, ab being perpendicular)
			// p is the point, thus ap in the vector between a and p
			tf::Vector3 ap(ptcloud_in->points[i].x - area_corners_[1][0],
						   ptcloud_in->points[i].y - area_corners_[1][1],
						   0.0);
			tf::Vector3 ab(area_corners_[0][0] - area_corners_[1][0],
						   area_corners_[0][1] - area_corners_[1][1],
						   0.0);
			tf::Vector3 ac(area_corners_[2][0] - area_corners_[1][0],
						   area_corners_[2][1] - area_corners_[1][1],
						   0.0);
			if (BETWEEN(ap.dot(ab), 0.0, ab.dot(ab)) &&
				BETWEEN(ap.dot(ac), 0.0, ac.dot(ac)))
			{
				point_count_++;
				//printf("Another point inside. Point count is %ld.\n", point_count_);
			}
		}
	}

	void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_in)
	{
		initial_pose_ = *pose_in;
	}

	void executeCB(const monarch_behaviors::CheckAreaClearGoalConstPtr &goal)
	{
		ros::Rate r(5);
		result_.area_empty = false;
		point_count_ = 0;

		initial_pose_.position.x = 1000.0;
		initial_pose_.position.y = 0.0;

		std::string robotid = action_name_.substr(1,6);
		std::stringstream ss;
		ss << "/sar/" << robotid << "/tfPose";
		ros::Subscriber own_pose_sub = nh_.subscribe<geometry_msgs::Pose>(ss.str(), 10, boost::bind(&CheckAreaClearAction::poseCallback, this, _1));

		while (initial_pose_.position.x == 1000.0) r.sleep();
		own_pose_sub.shutdown();

		area_corners_ = std::vector<tf::Vector3>();
		tf::Vector3 corner0(goal->area_points[0].x, goal->area_points[0].y, 0.0);
		area_corners_.push_back(corner0);
		tf::Vector3 corner1(goal->area_points[1].x, goal->area_points[1].y, 0.0);
		area_corners_.push_back(corner1);
		tf::Vector3 corner2(goal->area_points[2].x, goal->area_points[2].y, 0.0);
		area_corners_.push_back(corner2);

		lstpc_.subscribe_to_scan();

		for (unsigned int i = 0; i < goal->nav_points.size(); ++i)
		{
			move_base_msgs::MoveBaseGoal nextPoint;
			nextPoint.target_pose.header.frame_id = "/map";
			nextPoint.target_pose.header.stamp = ros::Time::now();
			nextPoint.target_pose.pose.position = goal->nav_points[i];

			geometry_msgs::Point current_position;
			if (i > 0)
			{
				current_position.x = goal->nav_points[i-1].x;
				current_position.y = goal->nav_points[i-1].y;
			}
			else
			{
				current_position.x = initial_pose_.position.x;
				current_position.y = initial_pose_.position.y;
			}

			double dx = goal->nav_points[i].x - current_position.x;
			double dy = goal->nav_points[i].y - current_position.y;
			double bearing;
			if ((dy == 0) && (dx == 0)) bearing = 0;
			else bearing = atan2(dy, dx);
			tf::Quaternion q;
			q.setRPY(0.0, 0.0, bearing);
			geometry_msgs::Quaternion heading;
			tf::quaternionTFToMsg(q, heading);
			nextPoint.target_pose.pose.orientation = heading;

			ac_nav_.sendGoal(nextPoint);

			while (ac_nav_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && 
					ac_nav_.getState() != actionlib::SimpleClientGoalState::ABORTED)
			{
				if (as_.isPreemptRequested() || !ros::ok())
				{
					ac_nav_.cancelAllGoals();
					as_.setPreempted(result_);
					lstpc_.unsubscribe_from_scan();
					return;
				}

				r.sleep();

			}

			if (ac_nav_.getState() == actionlib::SimpleClientGoalState::ABORTED)
			{
				as_.setAborted(result_);
				lstpc_.unsubscribe_from_scan();
				return;
			}
		}

		ROS_INFO("Area checking: Point count = %ld\n", point_count_);

		if(point_count_ < 1000) result_.area_empty = true;
		as_.setSucceeded(result_);
		lstpc_.unsubscribe_from_scan();
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "area_checker");

	CheckAreaClearAction checker(ros::this_node::getName());
	ros::spin();

	return 0;
}