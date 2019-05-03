#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
//JAC
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
// Costmap
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include"GlobalPlanner.h"

// Representation (RVIZ)
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>


tf::TransformListener *tf_ptr=NULL;
ros::Publisher *map_pose_pub_ptr=NULL;


void publishPath(ros::Publisher& path_pub, std::vector<geometry_msgs::PoseStamped>& plan )
{
	visualization_msgs::Marker marker;
  	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "path";
	marker.type = 4;
	marker.id = 0;
	marker.action = 0;

	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b =0;
	marker.color.a = 1;
	marker.scale.x = 0.05;
	for (unsigned i = 0; i< plan.size(); i++) {
		geometry_msgs::Point p;
		p.x = plan[i].pose.position.x;
		p.y =plan[i].pose.position.y;
		p.z = 0;
		marker.points.push_back(p);
	}
	path_pub.publish(marker);	


}


void odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	geometry_msgs::PoseStamped poseIn,poseOut;
	poseIn.pose = odom->pose.pose;
	poseIn.header = odom->header;	
	poseIn.header.stamp = ros::Time(0);
	try{
		tf_ptr->transformPose("map",poseIn,poseOut);
		poseOut.header.stamp = odom->header.stamp;
		map_pose_pub_ptr->publish(poseOut);	
	}catch(std::exception &e) {
		ROS_ERROR("%s",e.what());
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotics_challenge");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	double start_x, start_y;
	double goal_x, goal_y;

	tf::TransformListener tf(ros::Duration(10));
	tf_ptr = &tf;

  	costmap_2d::Costmap2DROS cost_map("cost_map", tf);

	// (start_x, start_y) are the coordinates of the initial position of the robot in MAP frame
	pn.param<double>("start_x",start_x,3.46);
	pn.param<double>("start_y",start_y,4.62);
	
	// (goal_x, goal_y) are the coordinates of the goal in MAP frame
	pn.param<double>("goal_x",goal_x,6.0);
	pn.param<double>("goal_y",goal_y,9.0);
	
	ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1, odomReceived);
	ros::Publisher map_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/map_pose", 1);
	map_pose_pub_ptr = &map_pose_pub;

	// Start position
  	geometry_msgs::PoseStamped start;
  	start.pose.position.x = start_x;
  	start.pose.position.y = start_y;
  	start.pose.position.z = 0.0;
  	start.header.stamp = ros::Time::now();
  
  	// Goal Position
  	geometry_msgs::PoseStamped goal;
  	goal.pose.position.x = goal_x;
  	goal.pose.position.y = goal_y;
  	goal.pose.position.z = 0.0;
  	goal.header.stamp = ros::Time::now();


	std::vector<geometry_msgs::PoseStamped> plan;
  	global_planner::GlobalPlanner planner("turtle_planner", &cost_map); 
  	planner.makePlan(start, goal, plan);	

	publishPath(path_pub,plan);
	ros::Rate loop_rate(15);

  	/**
  	* Complete the code to follow the path
  	*/

  	while (ros::ok())
  	{


	 ros::spinOnce();
       	 loop_rate.sleep();
  	}


	return 0;
}
