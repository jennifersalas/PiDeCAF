#ifndef PI_STANDARD_DEFS_H
#define PI_STANDARD_DEFS_H


#define INVALID_GPS_COOR -10000	//to indicate an ca_command to be ignored
				//NOTE: ca avoid is still responsible for outputting goal waypoint as needed
//Commands that GCS can send
#define EMERGENCY_PROTOCOL_LAT 999	//Set Latitude to this to indicate incoming gcs_command is a  meta message.
					//Set longitude to the following...
#define META_START_CA_ON_LON 555	//Start sending commands to ardupilot in Collision Avoid mode..
#define META_START_CA_OFF_LON 444	//Start sending commands to ardupilot in normal mode.
#define META_STOP_LON 777		//Turn collision avoidance off.

#endif	//PI_STANDARD_DEFS_H
