#include "au_uav_ros/collision_avoidance.h"

void au_uav_ros::CollisionAvoidance::init(int planeID)	{

	ROS_INFO("CollisionAvoidance:init()");
	me.setID(planeID);

}

//TODO: Add some method to not resend commands when the waypoint has not changed?
au_uav_ros::Command au_uav_ros::CollisionAvoidance::avoid(au_uav_ros::Telemetry telem)	{
	ROS_INFO("CollisionAvoidance::avoid() me position: %f, %f, %f | me destination: %f, %f", me.getCurrentLoc().latitude, me.getCurrentLoc().longitude, me.getCurrentLoc().altitude,
											me.getDestination().latitude, me.getDestination().longitude);
	au_uav_ros::Command newCmd;

	//Setting goalwp in plane object
	au_uav_ros::waypoint dest;
	goal_wp_lock.lock();
	dest.latitude = goal_wp.latitude;
	dest.longitude = goal_wp.longitude;
	dest.altitude = goal_wp.altitude;
	goal_wp_lock.unlock();
	me.setDestination(dest);

	au_uav_ros::waypoint tempForceWaypoint = fsquared::findTempForceWaypoint(me, telem);
	//Make new command from the calculated waypoint
	//newCmd.stamp = ros::Time::now();
	newCmd.planeID = me.getID();
	newCmd.commandID = 2;
	newCmd.param = 2;
	newCmd.latitude = tempForceWaypoint.latitude;
	newCmd.longitude = tempForceWaypoint.longitude;
	newCmd.altitude = me.getDestination().altitude;
	newCmd.replace = true;
	return newCmd;
}

void au_uav_ros::CollisionAvoidance::setGoalWaypoint(au_uav_ros::Command com)	{
	ROS_INFO("CollisionAvoidance::setGoalWaypoint()");
	
	goal_wp_lock.lock();
	goal_wp = com;
	goal_wp_lock.unlock();
}
