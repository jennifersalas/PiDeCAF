#ifndef MOVER_H 
#define MOVER_H 

/*
 * Mover node
 * Team 1 2013 REU
 *
 * Responsible for running collision avoidance and generating delicious new commands for the ardupilot.
 *
 * Subscribes to:
 * 	gcs_commands - Goal waypoint commands from ground control station.
 * 	all_telemetry - All telemetry messages from other planes and myself.
 *
 * Publishes to:
 * 	ca_commands - New collision avoidance waypoint commands for Ardupilot.
 */


#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

//collision avoidance library
#include "au_uav_ros/collision_avoidance.h"
#include "au_uav_ros/planeObject.h"

#include "au_uav_ros/pi_standard_defs.h"
#include "au_uav_ros/Fsquared.h"

//ros stuff
#include "ros/ros.h"
#include "au_uav_ros/Command.h"		
#include "au_uav_ros/Telemetry.h"
#include "au_uav_ros/planeIDGetter.h"	//Srv header that returns plane ID.


namespace au_uav_ros	{
	class Mover {
		private:
			/*
			 * For ease of testing this node by itself, the ros parameter "testing" can be set (see Parameter Server on wiki).
			 * In testing mode ... 
			 * 	the planeIDGetter service is not called 
			 * 	a fake planeID (999) is supplied 
			 * 	collision avoidance's avoid() function is faked to assign ca_wp to the goal waypoint. 
			 */
			bool is_testing;
			
			/*
			 * Statefulness
			 * Mover node operates in three states. 
			 * Motivation: The Ardupilot may not respond to remote control if commands are being continually sent. 
			 * 	ST_RED  		STOP. Do not publish any ca_commands. 
			 * 	ST_GREEN_CA_ON 		Publish to ardu through ca_commands, with collision avoidance turned on. 
			 * 	ST_GREEN_CA_OFF 	Publish to ardu through ca_commands, with collision avoidance turned off.
			 * 				Will publish goal waypoints. 
			 *
			 * Changing States: 	Commands whose latitude and longitude are specific values will trigger a state change.
			 * 			See pi_standard_defs.h for details.
			 */
			enum state {ST_RED, ST_GREEN_CA_ON, ST_GREEN_CA_OFF};	
			enum state current_state; 
			boost::mutex state_change_lock;			//Both gcs_command callback and move() need to fight over access to this state.

			/*
			 * Collision Avoidance
			 * This object holds the collision avoidance algorithm itself. Mover will call ca.avoid() on every telemetry update
			 * from other planes and my own plane, which returns a command to go to.
			 * Assumption: 	When state is ST_GREEN_CA_ON, ca is responsible for all logic related to generating correct next waypoint.
			 * 		The only responsiblity of Mover is to continually send out the first waypoint in ca_wp deque at the
			 * 		rate specified in move(). If ca_wp is empty, no command is sent. 
			 */
			CollisionAvoidance ca;

			int planeID;					//current plane id
			float initialLong, initialLat, initialAlt;	//First telemetry update, used to initialize goal_wp
			
			//Queues for Waypoints
			au_uav_ros::Command goal_wp;			//store goal wp from Ground control 
			std::deque<au_uav_ros::Command> ca_wp;		//store waypoint to go to next, produced by collision avoidance 

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
			 * Replaces ca_wp with avoid()'s returned command.
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

			// Publish commands in state (ST_GREEN_CA_ON) 
			void caCommandPublish();
			// Publish goal_wp in state (ST_GREEN_CA_OFF) 
			void goalCommandPublish();
		public:
			int getPlaneID() {return planeID;} 
			bool init(ros::NodeHandle n, bool testing);
			void run();

	};
}	


#endif	//COLLISION_AVOIDANCE_LOGIC_H
