#include "au_uav_ros/gcs_talker.h"
#include <cstdio> //printf
#include <string>
#include <ros/console.h>	//used for debugging

au_uav_ros::GCSTalker::GCSTalker()	{
	m_port = "dev/ttyUSB0";
	m_baud = 57600;
}

au_uav_ros::GCSTalker::GCSTalker(std::string _port, int _baud)	{
	m_port = _port;
	m_baud = _baud;	
	
}

bool au_uav_ros::GCSTalker::init(ros::NodeHandle _n)	{
	//Open and setup port.
	if(xbee_talker.open_port(m_port) == -1)	{
		ROS_INFO("Could not open port %s", m_port.c_str());
		return false;
	}
	else	{
		ROS_INFO("opened port %s", m_port.c_str());
		xbee_talker.setup_port(m_baud, 8, 1, true);
	}

	//mavlink
	sysid = -1;
	compid = 110;	//not entirely sure why this is 110
	serial_compid =0;	//the compid that the GCS is getting from plane (so we won't need)
	pc2serial = true;
	updateIndex = 0;
	WPSendSeqNum = 0;

	//Set up Ros stuff. Todo - 
	m_node = _n;
	m_command_sub = m_node.subscribe("gcs_commands", 10, &GCSTalker::commandCallback, this);
	telem_sub = m_node.subscribe("my_mav_telemetry", 2, &GCSTalker::myTelemCallback, this);
	m_telem_pub = m_node.advertise<au_uav_ros::Telemetry>("all_telemetry", 5);
	m_mav_telem_pub = m_node.advertise<au_uav_ros::Telemetry>("my_mav_telemetry", 5);
	return true;
}

void au_uav_ros::GCSTalker::run()	{
	ROS_INFO("Entering Run");

	//Spin up thread to execute myTelemCallback()
	boost::thread sendCommands(boost::bind(&GCSTalker::spinThread, this));

	//Start listening for telemetry and commands, upon receiving proper command
	//from ground station, execute shutdown and join
	listen();

	ros::shutdown();
	sendCommands.join();	
}

void au_uav_ros::GCSTalker::shutdown()	{
	//ROS_INFO("Shutting down Xbee port %s", m_port.c_str());
	printf("GCSTalker::Shutting down Xbee port %s\n", m_port.c_str()); //i ros::shutdown when exiting run
	xbee_talker.close_port();
}

//Input - listening for other telem msgs
//---------------------------------------------------------------------------

void au_uav_ros::GCSTalker::listen()	{
	while(ros::ok())
	{
		//get a mavlink message from the serial line
		mavlink_message_t message = au_uav_ros::mav::readMavlinkFromSerial(xbee_talker);
		//decode the message and post it to the appropriate topic
		if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
		{
			mavlink_heartbeat_t receivedHeartbeat;
			mavlink_msg_heartbeat_decode(&message, &receivedHeartbeat);
			ROS_INFO("Received heartbeat");
		}
		if(message.msgid == MAVLINK_MSG_ID_AU_UAV)
		{
			ROS_INFO("Received AU_UAV message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
			au_uav_ros::Telemetry tUpdate, tRawUpdate;
			mavlink_au_uav_t myMSG;
			mavlink_msg_au_uav_decode(&message, &myMSG);
				
			//Post update as new telemetry update
			au_uav_ros::mav::convertMavlinkTelemetryToROS(myMSG, tUpdate);
			tUpdate.planeID = message.sysid;
			m_telem_pub.publish(tUpdate);
			ROS_INFO("Received telemetry message from UAV[#%d] (lat:%f|lng:%f|alt:%f)", tUpdate.planeID, tUpdate.currentLatitude, tUpdate.currentLongitude, tUpdate.currentAltitude);

			if(message.sysid != 5){
				//Forward raw telemetry update to the xbee_talker node
				au_uav_ros::mav::rawMavlinkTelemetryToRawROSTelemetry(myMSG, tRawUpdate);
				tRawUpdate.planeID = message.sysid;
				m_mav_telem_pub.publish(tRawUpdate);
			}
		}
	}
}



//Output - writing to xbee
//---------------------------------------------------------------------------
void au_uav_ros::GCSTalker::commandCallback(au_uav_ros::Command cmd)	{
	ROS_INFO("GCSTalker::commandCallback::ding! \n");
	//----------------
	//Callback time! 
	//----------------
	mavlink_message_t mavlinkMsg;
	sysid = cmd.planeID;
	//stuff mavlinkMsg with the correct paramaters
	mavlink_msg_mission_item_pack(cmd.planeID, compid, &mavlinkMsg, 
			              cmd.planeID, serial_compid, 0, 
				      MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 
				      2, 0, 20.0, 100.0, 1.0, 0.0, 
				      cmd.latitude, cmd.longitude, cmd.altitude);

	//take command -> send to ardupilots
	static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int messageLength = mavlink_msg_to_send_buffer(buffer, &mavlinkMsg);
	xbee_talker.lock();
	int written = write(xbee_talker.getFD(), (char*)buffer, messageLength);
	xbee_talker.unlock();
	if (messageLength != written) ROS_ERROR("ERROR: Wrote %d bytes but should have written %d\n",
						written, messageLength);
}

void au_uav_ros::GCSTalker::myTelemCallback(au_uav_ros::Telemetry tUpdate)	{
	//Write that stuff out to the xbee!!
	ROS_INFO("GCSTalker::telemCallback::ding! \n");

	mavlink_message_t mavlinkMsg;
		static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	//stuff mavlinkMsg with all the correct paramaters
	mavlink_msg_au_uav_pack(tUpdate.planeID, compid, &mavlinkMsg, tUpdate.currentLatitude, tUpdate.currentLongitude,
	tUpdate.currentAltitude, tUpdate.destLatitude, tUpdate.destLongitude,
					tUpdate.destAltitude, tUpdate.groundSpeed, tUpdate.airSpeed, tUpdate.targetBearing,
	tUpdate.distanceToDestination, tUpdate.currentWaypointIndex);


		int messageLength = mavlink_msg_to_send_buffer(buffer, &mavlinkMsg);
	ros::Duration(0.000001).sleep();
		xbee_talker.lock();
		int written = write(xbee_talker.getFD(), (char*)buffer, messageLength);
		xbee_talker.unlock();
		if (messageLength != written) ROS_ERROR("ERROR: Wrote %d bytes but should have written %d\n",
							written, messageLength);

}

void au_uav_ros::GCSTalker::spinThread()	{
	ROS_INFO("GCSTalker::spinThread::Starting spinner thread");
	//Handle myTelemCallback()
	ros::spin();	
}

int main(int argc, char** argv)	{

	std::cout << "hello world!" <<std::endl;
	std::string port = "/dev/ttyUSB0";
	if(argc > 1)
		port.assign(argv[1]);
	ros::init(argc, argv, "GCSTalker");
	ros::NodeHandle n;
	au_uav_ros::GCSTalker talk(port, 57600);
	if(talk.init(n))	//OK. what if the xbee can't be read???
		talk.run();
	talk.shutdown();
}
