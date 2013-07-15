#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include "ros/ros.h"

#include <boost/thread/mutex.hpp>

//Custom ros msgs
#include "au_uav_ros/Telemetry.h"
#include "au_uav_ros/Command.h"

#include "au_uav_ros/pi_standard_defs.h"
#include "au_uav_ros/planeObject.h"
#include "au_uav_ros/Fsquared.h"

namespace au_uav_ros	{
	class CollisionAvoidance	{
	private:
		au_uav_ros::PlaneObject me;  	//planeObject representation of the plane running this algorithm
		au_uav_ros::Command goal_wp;

		boost::mutex goal_wp_lock;	//coordinate access to goal_wp 
	public:
		void init(int planeID);	
		/*
		 * Called by mover's Telem callback. Takes in all telemetry callbacks (including my own).
		 * Returns desired command, with bool replace field indicating wheter to queue up or replace with new CA waypoint.
		 * If Command's lat, long, and alt fields are INVALID_GPS_COOR, it will be ignored.
		 */
		au_uav_ros::Command avoid(au_uav_ros::Telemetry telem);	//Called when there's a telemetry callback.

		/*
		 * When mover receives a new GCS command, this function will be called.
		 * Updates CA's goal waypoint to match mover's
		 */
		void setGoalWaypoint(au_uav_ros::Command com);
	};
}

#endif
