#ifndef GCS_TALKER_H
#define GCS_TALKER_H


#include <errno.h>
#include <boost/thread.hpp>

//ros stuff
#include "au_uav_ros/serial_talker.h"
#include "au_uav_ros/mavlink_read.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <au_uav_ros/Telemetry.h>
#include <au_uav_ros/Command.h>
//mavlink stuff
#include "mavlink/v1.0/ardupilotmega/mavlink.h"
/*
 * Possible bug. Unable to contact roscore, need to put in provision?
 *
 * What to do if - can't open port
 * 		 - can open port, but need to close? right now only closes on receiving a 'q'
 * 		 
 */


namespace au_uav_ros{
	class GCSTalker	{
	private:
		SerialTalker m_gcs;
		std::string m_port;
		int m_baud;

		//mavlink stuff
		int sysid;
		int compid;	//i have no idea what these do
		int serial_compid;
		bool pc2serial;	
		int updateIndex;
		int WPSendSeqNum;

		//ros stuff
		ros::NodeHandle m_node;
		ros::Subscriber m_command_sub;	//Subscribes to CA commands 
		ros::Subscriber telem_sub;
		ros::Publisher m_telem_pub;
		ros::Publisher m_mav_telem_pub;
	public:
		GCSTalker();
		GCSTalker(std::string port, int baud);
		
		bool init(ros::NodeHandle _n);	//Opens  and sets up port, sets up ros stuff .
		void run();
		void shutdown();
	
		//In - reading from gcs/listening
		void listen();

		//Out - writing to ardu 
		void spinThread();				//spin() and listens for myTelemCallbacks
		void commandCallback(au_uav_ros::Command cmd);	//sends commands to planes in air
		void myTelemCallback(au_uav_ros::Telemetry tUpdate);
	};
}
#endif
