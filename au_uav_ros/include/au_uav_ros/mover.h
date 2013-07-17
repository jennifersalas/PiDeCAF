
#ifndef MOVER_H 
#define MOVER_H 


#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

//collision avoidance library
#include "au_uav_ros/collision_avoidance.h"
#include "au_uav_ros/planeObject.h"

#include "au_uav_ros/planeIDGetter.h"

#include "au_uav_ros/pi_standard_defs.h"
#include "au_uav_ros/Fsquared.h"

//ros stuff
#include "ros/ros.h"
#include "au_uav_ros/Command.h"
#include "au_uav_ros/Telemetry.h"


namespace au_uav_ros	{
	class Mover {
		private:
			
			//Meta - State Machine fun. NOte - State is changed by gcs_command callback.
			enum state {ST_RED, ST_GREEN_CA_ON, ST_GREEN_CA_OFF, ST_GRADIENT_TELEM, ST_GRADIENT_AVOID};	
			enum state current_state; 
			boost::mutex state_change_lock;			//Both gcs_command callback and move() need to FIGHT TO THE DEATH

			//Collision Avoidance fun
			CollisionAvoidance ca;

			int planeID;					//current plane id
			float initialLong, initialLat, initialAlt;	//First telemetry update, used to initialize goal_wp
			
			//Queues for Waypoints
			au_uav_ros::Command goal_wp;			//store goal wp from Ground control 
			std::deque<au_uav_ros::Command> ca_wp;	 	//store collision avoidance waypoints 

			//Locks for Queues
			boost::mutex goal_wp_lock;
			boost::mutex ca_wp_lock;

			//ROS stuff
			ros::NodeHandle nh;
			ros::ServiceClient IDclient;	//Get my plane's ID form ardupilot node.
			ros::Publisher ca_commands;	//Publish actual CA command waypoints
			ros::Subscriber my_telem_sub;	//Subscribe to just me telemetry (in raw mav format)
			ros::Subscriber all_telem;	//Subscribe to all telemetry msgs (me and other planes)
			ros::Subscriber gcs_commands;	//Subscribe to commands from Ground control

			/*
			 * Callback for any incoming telemetry msg (including my own).
			 * Updates my position if it's my telemetry.
			 * Calls CollisionAvoidance's avoid() function.
			 * Replaces or queues up avoid()'s returned command.
			 */
			void all_telem_callback(au_uav_ros::Telemetry telem);

			
			/*
			 * callback for any ground control commands.
			 * Replaces current goal wp with incoming command.
			 */ 
			void gcs_command_callback(au_uav_ros::Command com);

			//thread stuff, calls ros::spin()
			void spinThread();

			//main decision making logic
			void move();

			//OK you can publish CA commands (ST_GREEN_CA_ON).
			void caCommandPublish();
			//OK you can publish goal commands, ignore CA. (ST_GREEN_CA_OFF).
			void goalCommandPublish();
		public:
			int getPlaneID() {return planeID;} 
			bool init(ros::NodeHandle n);
			void run();

	};
}	


#endif	//COLLISION_AVOIDANCE_LOGIC_H
