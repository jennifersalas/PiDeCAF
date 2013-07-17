#include "au_uav_ros/mover.h"

//callbacks
//----------------------------------------------------
void au_uav_ros::Mover::all_telem_callback(au_uav_ros::Telemetry telem)	{

	//CA will go here.
	//It's OK to have movement/publishing ca-commands here, since this will be called
	//when ardupilot publishes *my* telemetry msgs too.
	
	//get state for testing purposes
	enum state temp;
	state_change_lock.lock();
	temp = current_state;
	state_change_lock.unlock();

	au_uav_ros::Command com;
	switch(temp)	{
		case(ST_GRADIENT_TELEM):	
			fprintf(stderr, "\nmover::telem_callback gradient_telem(%f|%f|%f)\n", com.latitude, com.longitude, com.altitude);
			goal_wp_lock.lock();
			com = goal_wp;	
			goal_wp_lock.unlock();
			break;
		case(ST_GRADIENT_AVOID):
			fprintf(stderr, "\nmover::telem_callback gradient_avoid(%f|%f|%f)\n", com.latitude, com.longitude, com.altitude);
			com = ca.avoid(telem, true);
			break;
		case(ST_GREEN_CA_ON):
			fprintf(stderr, "\nmover::telem_callback ca_on(%f|%f|%f)\n", com.latitude, com.longitude, com.altitude);
			com = ca.avoid(telem, false);
			break;
	}

	//Check if ca_waypoint should be ignored
	if(com.latitude == INVALID_GPS_COOR && com.longitude == INVALID_GPS_COOR && com.altitude == INVALID_GPS_COOR)	{
		//ignore.
	}		
	else	{
		ca_wp_lock.lock();
		ca_wp.clear();
		ca_wp.push_back(com);	
		ca_wp_lock.unlock();	
	}
}

void au_uav_ros::Mover::gcs_command_callback(au_uav_ros::Command com)	{

	//State changing will be done in this function, instead of in move(). This GCS callback will be executed not as frequently as the move,
	//since not many gcs commands will be coming in.

	//TESTING STUFF - Quick Emergency Protocol - START and STOP publishing to ca_commands to prevent overtaking manual mode.
	if(planeID == com.planeID)	{
		enum state temp;
		if(com.latitude == EMERGENCY_PROTOCOL_LAT)	{
			int incomingCommand = (int)com.longitude;
			switch(incomingCommand)	{
			case(META_START_CA_ON_LON):	{	
				int gradient = (int)com.altitude;
				switch(gradient)	{
					case(META_GRADIENT_TELEM_ALT):	{
						fprintf(stderr, "Mover::CHANGING TO GRADIENT_TELEM MODE\n");
						state_change_lock.lock();
						current_state = ST_GRADIENT_TELEM;
						state_change_lock.unlock();
						break;
					}
					case(META_GRADIENT_AVOID_ALT):	{
						fprintf(stderr, "Mover::CHANGING TO GRADIENT_AVOIDMODE\n");
						state_change_lock.lock();
						current_state = ST_GRADIENT_AVOID;
						state_change_lock.unlock();
						break;
					}
					default:	{	
						fprintf(stderr, "Mover::CHANGING TO GREEN CA ON MODE\n");
						//change state to using CA 
						state_change_lock.lock();
						current_state = ST_GREEN_CA_ON;
						state_change_lock.unlock();
					}
				}
				break;
			}
			//No matter the state, STOP publishing.
			case(META_STOP_LON):	
				fprintf(stderr, "Mover::CHANGING TO NOGO MODE\n");
				//change state to NOGO
				state_change_lock.lock();
				current_state = ST_RED;
				state_change_lock.unlock();
				break;
			case(META_START_CA_OFF_LON):
				fprintf(stderr, "Mover::CHANGING TO GREEN CA OFF MODE\n");
				//change state to not using CA 
				state_change_lock.lock();
				current_state = ST_GREEN_CA_OFF;
				state_change_lock.unlock();
				break;
			}
		}
		else	{
			//just a regular old gcs command, move along now.
			//Add this to the goal_wp q.
			goal_wp_lock.lock();
			goal_wp = com;	
			goal_wp_lock.unlock();
//			ROS_INFO("Received new command with lat%f|lon%f|alt%f", com.latitude, com.longitude, com.altitude);
			fprintf(stderr, "mover::callback::Received new command with lat%f|lon%f|alt%f", com.latitude, com.longitude, com.altitude);
			ca.setGoalWaypoint(com);

		}	
	}
}

//node functions
//----------------------------------------------------

bool au_uav_ros::Mover::init(ros::NodeHandle n)	{
	//Ros stuff
	nh = n;

	IDclient = nh.serviceClient<au_uav_ros::planeIDGetter>("getPlaneID");
	ca_commands = nh.advertise<au_uav_ros::Command>("ca_commands", 10);	
	all_telem = nh.subscribe("all_telemetry", 10, &Mover::all_telem_callback, this);
	gcs_commands = nh.subscribe("gcs_commands", 10, &Mover::gcs_command_callback, this);	
	

	//Find out my Plane ID.
	//-> need timed call? or keep trying to get plane id???
	bool fake_id;
	n.param<bool>("fake_id", fake_id, false);
	if(fake_id)
		planeID = 999;
	else	{
		au_uav_ros::planeIDGetter srv;
		if(IDclient.call(srv))	{
			ROS_INFO("mover::init Got plane ID %d", srv.response.planeID);
			ROS_INFO("mover::init Got initial position lat: %f|long: %f|alt: %f",
					srv.response.initialLatitude, srv.response.initialLongitude, srv.response.initialAltitude);
			planeID = srv.response.planeID;
			goal_wp.planeID = planeID;
			goal_wp.latitude = initialLat = srv.response.initialLatitude;
			goal_wp.longitude= initialLong = srv.response.initialLongitude;
			goal_wp.latitude = initialAlt = srv.response.initialLatitude;
			goal_wp.param = 2;
			goal_wp.commandID = 2;
		}
		else	{
			ROS_ERROR("mover::init Unsuccessful get plane ID call.");//what to do here.... keep trying?
			return false;
		}
	}
	//CA init
	ca.init(planeID);

	current_state = ST_RED;
	return true;
}

void au_uav_ros::Mover::run()	{

	ROS_INFO("Entering mover::run()");

	//spin up thread to get callbacks
	boost::thread spinner(boost::bind(&Mover::spinThread, this));

	//Given GCS commands and ca waypoints, decide which ones to send to ardupilot	
	move();

	ros::shutdown();
	spinner.join();
}



void au_uav_ros::Mover::move()	{

	ROS_INFO("Entering mover::move()");	
	au_uav_ros::Command com;
	
	while(ros::ok())	{
		//state machine fun
		//note - current state is changed in gcs_callback
		//First, get the current state
		enum state temp;
		state_change_lock.lock();
		temp = current_state;
		state_change_lock.unlock();
		//then do some switching
		switch(current_state)	{
			case(ST_RED):
				//DO NOT PUBLISH! DO NOT PUBLISH!
				break;
			case(ST_GREEN_CA_OFF):
				ros::Duration(0.25).sleep(); 	//no swamping the ardupilot, it's a delicate thing 
				goalCommandPublish();
				break;
			case(ST_GRADIENT_TELEM):
				fprintf(stderr, "mover::(ST_GRADIENT_TELEM)");
			case(ST_GRADIENT_AVOID):
				fprintf(stderr, "mover::(ST_GRADIENT_AVOID)");
			case(ST_GREEN_CA_ON):
				fprintf(stderr, "mover::(ST_GREEN_CA_ON) PUBLISHING CA COMMAND!\n");
				ros::Duration(0.25).sleep(); 	//no swamping the ardupilot, it's a delicate thing 
				caCommandPublish();
				break;
		}
		
	}
}

//Assumes we are in ST_GREEN_CA_OFF
void au_uav_ros::Mover::goalCommandPublish()	{
	fprintf(stderr, "mover::(ST_GREEN_CA_OFF) PUBLISHING goal COMMAND!\n");
	au_uav_ros::Command com;

	//Just send out goal wp, no collision avoidance.
	goal_wp_lock.lock();
	com = goal_wp;	
	goal_wp_lock.unlock();
	ca_commands.publish(com);

}

//Assumes we are in ST_GREEN_CA_ON mode.
void au_uav_ros::Mover::caCommandPublish()	{
	au_uav_ros::Command com;
	
	bool empty_ca_q = false;
	ca_wp_lock.lock();
	empty_ca_q = ca_wp.empty();
	if(!empty_ca_q)	{
		com = ca_wp.front();
		ca_wp.pop_front();	
	}
	ca_wp_lock.unlock();	
	//don't want to forward if no ca command is returned
	if(!empty_ca_q)
		ca_commands.publish(com);
}

void au_uav_ros::Mover::spinThread()	{
	ROS_INFO("mover::starting spinner thread");
	ros::spin();
}
//main
//----------------------------------------------------
int main(int argc, char **argv)	{
	ros::init(argc, argv, "ca_logic");
	ros::NodeHandle n;

	bool is_test;
//	n.param<bool>("testing", is_test, false);
	au_uav_ros::Mover mv;
	
	fprintf(stderr, "MOVER::Testing var is %d", (int)is_test);
	
	if(mv.init(n ))	//channnge if doing real planes to true
		mv.run();	
	//spin and do move logic in separate thread

}


