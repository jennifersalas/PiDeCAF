//AT LEAST a 1.2 second delay when it stops sending commands
//let's test mover. make sure that thing WORKS


#include <vector>
#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <au_uav_ros/pi_standard_defs.h>
#include <roscpp/Logger.h>
#include <au_uav_ros/Command.h>
#include <au_uav_ros/Telemetry.h>

#include <boost/thread.hpp>

class test_state_machine {
	public:
	
	boost::mutex queue_lock;

	//stuff
	ros::Time sinceLastCACommand;	
	ros::Duration lag;	//one second lag OK from sending command to reponse

	au_uav_ros::Command stop;
	au_uav_ros::Command go_ca_off;
	au_uav_ros::Command go_ca_on;
	au_uav_ros::Command regCom;
	au_uav_ros::Command gradient_telem;
	au_uav_ros::Command gradient_avoid;

	au_uav_ros::Command previousCom;			//don't want to pushback duplicate commands

	std::vector<au_uav_ros::Command> ca_commands_bin;	// all the ca waypoints i've gotten.
	std::vector<ros::Time> time_bin;			// time that ca_waypoints came in. 
	double sleep_time; 

	ros::Duration actual_lag;

	test_state_machine()	{
		sinceLastCACommand = ros::Time::now();
		lag.sec = 1;
		sleep_time = 1.5;
		stop.latitude = EMERGENCY_PROTOCOL_LAT;
		stop.longitude = META_STOP_LON;
		stop.planeID = 999;

		go_ca_off.latitude = EMERGENCY_PROTOCOL_LAT;
		go_ca_off.longitude = META_START_CA_OFF_LON;
		go_ca_off.planeID = 999;
		
		go_ca_on.latitude = EMERGENCY_PROTOCOL_LAT;
		go_ca_on.longitude = META_START_CA_ON_LON;
		go_ca_on.planeID = 999;

		gradient_telem.latitude = EMERGENCY_PROTOCOL_LAT;
		gradient_telem.longitude = META_START_CA_ON_LON;
		gradient_telem.altitude = META_GRADIENT_TELEM_ALT;
		gradient_telem.planeID = 999;	

		gradient_avoid.latitude = EMERGENCY_PROTOCOL_LAT;
		gradient_avoid.longitude = META_START_CA_ON_LON;
		gradient_avoid.altitude = META_GRADIENT_AVOID_ALT;
		gradient_avoid.planeID = 999;	

		regCom.latitude = 0;
		regCom.longitude = 0;
		regCom.planeID = 999;
	}


	void ca_command_callback(au_uav_ros::Command com)	{
		ros::Time arrival = ros::Time::now();
		fprintf(stderr, "tester::GOT CA COMMAND CALLBACK!");	
		fprintf(stderr, "\traw:: lat: %f|lon: %f|alt: %f\n",com.latitude, com.longitude, com.altitude);
		sinceLastCACommand = ros::Time::now();	

		
		//Only push back if it's CHANGED (alg continually outputs goal_wp)  and not default (0.0)
		if(com.latitude != 0 && previousCom.latitude != com.latitude)	{ 
			fprintf(stderr, "tester::callback -> new ca_command new new new!\n");
			fprintf(stderr, "\tnew:: lat: %f|lon: %f|alt: %f\n\n",com.latitude, com.longitude, com.altitude);
			queue_lock.lock();
			ca_commands_bin.push_back(com);
			time_bin.push_back(arrival);	
			queue_lock.unlock();
		}
		previousCom = com;
	}

	
	std::vector<au_uav_ros::Command> generateFakeWaypoints(int numWaypoints)	{

		
		au_uav_ros::Command com;
		com.latitude = 10; com.longitude = 10; com.altitude = 10; com.planeID = 999;	
		std::vector<au_uav_ros::Command> forceFeeding;
		for(int i =0; i< numWaypoints; i++)	{
			forceFeeding.push_back(com);
			com.latitude++; com.longitude++; com.altitude++;
			fprintf(stderr, "test::Pushed back lat: %f| long: %f| alt: %f\n", forceFeeding[i].latitude,
											forceFeeding[i].longitude,
											forceFeeding[i].altitude);

		}
		return forceFeeding;

	} 

	std::vector<ros::Time> sendGCSCommands(std::vector<au_uav_ros::Command> forceFeeding, ros::Publisher gcs)	{
	
		std::vector<ros::Time> sendingTimes;
		for(int i =0; i< forceFeeding.size(); i++)	{
			fprintf(stderr, "test::GCS sent lat: %f| long: %f| alt: %f\n", forceFeeding[i].latitude,
											forceFeeding[i].longitude,
											forceFeeding[i].altitude);
			gcs.publish(forceFeeding[i]);
			sendingTimes.push_back(ros::Time::now());	
			ros::Duration(.25).sleep();	//time enough to measure lag initially
		}

		return sendingTimes;

	
	}



	bool ca_commands_chatty()	{
		actual_lag = ros::Time::now() - sinceLastCACommand;
		return actual_lag < lag; 
	}

	bool ca_commands_silent()	{
		actual_lag = ros::Time::now() - sinceLastCACommand;
		return actual_lag > lag; 
	}

	std::vector<au_uav_ros::Command> getReceivedCommands()	{
		queue_lock.lock();
		std::vector<au_uav_ros::Command> temp(ca_commands_bin);
		queue_lock.unlock();
		return temp;
	}
	std::vector<ros::Time> getReceivedTimes()	{
		queue_lock.lock();
		std::vector<ros::Time> temp(time_bin);
		queue_lock.unlock();
		return temp;
	}
};

void spinThread()	{
	fprintf(stderr, "ros::spin spinning...\n");
	ros::spin();
}

//Publish my own fake telemetry. Force ca.avoid to be called
void pubTelem()	{
	fprintf(stderr, "pub telem thread:: spinning... \n");
	ros::Rate r(4);	//publish telem - this rate WILL end in dropped :w


	//ros publish
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<au_uav_ros::Telemetry>("all_telemetry",5);
	au_uav_ros::Telemetry telem;
	while(ros::ok())	{
		pub.publish(telem);	
		r.sleep();	
	}
}

TEST(mover, amTesting)	{
	ros::NodeHandle n;
	bool t;
	n.getParam("testing", t);
	ASSERT_TRUE(t);
}


TEST(mover, state_runthrough)	{


	test_state_machine t;
	ros::NodeHandle n;
	ros::Publisher gcs = n.advertise<au_uav_ros::Command>("gcs_commands", 5);	
	
	ros::Subscriber ca = n.subscribe("ca_commands", 1, &test_state_machine::ca_command_callback, &t);	
	
	ros::Publisher fake = n.advertise<au_uav_ros::Command>("ca_commands", 5);	
	fake.publish(t.regCom);
	fprintf(stderr, "test::FAKE PUBLISH");
	ros::Duration(5).sleep();	//time enough to measure lag initially

	//NO GO MODE
	//-------------
	//Mover should be starting in NOGO mode.
	fprintf(stderr, "TEST::Expect start in NOGO\n");
	EXPECT_TRUE(t.ca_commands_silent());	//No commands should have been sent within the last lag period.

	//RED...

	gcs.publish(t.stop); //publish stop...
	EXPECT_TRUE(t.ca_commands_silent()) << "RED START | stop should be silent";	

	gcs.publish(t.regCom);	//publish regular command
	EXPECT_TRUE(t.ca_commands_silent()) << "RED START | regCom  should be silent";	

	//RED->GREEN_CA_OFF
	gcs.publish(t.go_ca_off); //transition
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty()) << "RED -> GREEN_CA_OFF | CA_OFF should be chatty" ;	

	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_chatty()) << "GREEN_CA_OFF | regCom should be chatty";	

	//GREEN_CA_OFF -> GREEN_CA_ON
	gcs.publish(t.go_ca_on);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty()) << "GREEN_CA_OFF -> GREEN_CA_ON | CA_ON should be chatty" ;
	
	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_chatty()) << "GREEN_CA_ON | regCom should be chatty";	

	//GREEN_CA_ON -> RED
	gcs.publish(t.stop);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent()) << "GREEN_CA_ON -> RED| stop should be silent";		
	
	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_silent()) << "RED | regCom should be silent";	

	//RED->GREEN_CA_ON
	gcs.publish(t.go_ca_on);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty()) << "RED -> GREEN_CA_ON | CA_ON should be chatty";	
	
	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_chatty()) << "GREEN_CA_ON | regCom should be chatty";	

	//GREEN_CA_ON -> GREEN_CA_OFF
	gcs.publish(t.go_ca_off); //transition
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty()) << "GREEN_CA_ON -> GREEN_CA_OFF | caOff should be chatty";	

	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_chatty()) << "GREEN_CA_OFF| regCom should be chatty";	

	//GREEN_CA_OFF -> RED
	gcs.publish(t.stop); //publish stop... transition back
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent()) << "GREEN_CA_OFF -> RED | stop should be silent";	


	//RED-> GRAD_TELEM
	gcs.publish(t.gradient_telem);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty()) << "RED -> GRAD_TELEM| GRAD_TELEM should be chatty";	
	
	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_chatty()) << "GRAD_TELEM| regCom should be chatty";	

	//GRAD_TELEM -> GRAD_AVOID
	gcs.publish(t.gradient_avoid);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty()) << "GRAD_TELEM-> GRAD_AVOID| GRAD_AVOID should be chatty" ;
	
	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_chatty()) << "GRAD_AVOID| regCom should be chatty";	

	//GRAD_AVOID-> GREEN_CA_ON
	gcs.publish(t.go_ca_on);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty()) << "GRAD_AVOID-> CA_ON| CA_ONshould be chatty" ;
	
	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_chatty()) << "CA_ON| regCom should be chatty";	


	//GREEN_CA_ON -> RED
	gcs.publish(t.stop);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent()) << "GREEN_CA_ON -> RED| stop should be silent";		
	
	gcs.publish(t.regCom);
	EXPECT_TRUE(t.ca_commands_silent()) << "RED | regCom should be silent";	


}


//Given that avoidance is turned off, feed goal wps to mover.
//om nom nom

TEST(mover, ca_off_feeding)	{

	
	
	test_state_machine t;
	ros::NodeHandle n;	
	int numWaypoints = 5;
	ros::Publisher gcs = n.advertise<au_uav_ros::Command>("gcs_commands", 5);	
	ros::Subscriber ca = n.subscribe("ca_commands", 10, &test_state_machine::ca_command_callback, &t);	

	//Transitioning to ca_off state	
	fprintf(stderr, "-> GREEN_CA_OFF/n");
	gcs.publish(t.stop);
	ros::Duration(7).sleep(); //let spinner thread startup 
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent()) << "actuallag: s:" << t.actual_lag.sec << " ns:" << t.actual_lag.nsec;
	gcs.publish(t.go_ca_off); //transition
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty());	

	//Publishing some GCS commands
	std::vector<au_uav_ros::Command> forceFeeding = t.generateFakeWaypoints(5);
	std::vector<ros::Time> sendingTimes = t.sendGCSCommands(forceFeeding, gcs);


	gcs.publish(t.regCom);			//goal_wp back to 0|0|0 for next test

	fprintf(stderr, "STOP STOP!\n");	
	gcs.publish(t.stop); //publish stop... transition back
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	

	//Now, seeing if the bins are equal...

	fprintf(stderr, "Trying to get received...\n");	
	std::vector<au_uav_ros::Command> ca_commands_bin = t.getReceivedCommands();	
	std::vector<ros::Time> time_bin = t.getReceivedTimes();		

	
	for(int i =0; i< numWaypoints; i++)	{
		fprintf(stderr, "received :: back lat: %f \n", ca_commands_bin[i].latitude);
		EXPECT_TRUE(forceFeeding[i].latitude == ca_commands_bin[i].latitude); 
		EXPECT_TRUE(forceFeeding[i].longitude == ca_commands_bin[i].longitude);
		EXPECT_TRUE(forceFeeding[i].altitude == ca_commands_bin[i].altitude) << "differ: " << forceFeeding[i] << "and " << ca_commands_bin[i]; 

		fprintf(stderr, " Time diff (s) command %d: %f\n\n", i, (time_bin[i] - sendingTimes[i]).toSec());	
	}		



}


TEST(mover, gradient_avoid_feeding)	{


	test_state_machine t;
	ros::NodeHandle n;	
	int numWaypoints = 5;
	ros::Publisher gcs = n.advertise<au_uav_ros::Command>("gcs_commands", 5);	
	ros::Subscriber ca = n.subscribe("ca_commands", 10, &test_state_machine::ca_command_callback, &t);	
	

	//Transitioning to ca_on state	
	fprintf(stderr, "-> GRADIENT_AVOID\n");
	gcs.publish(t.stop);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	
	gcs.publish(t.gradient_avoid); //transition
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty());	

	//Publishing some GCS commands
	std::vector<au_uav_ros::Command> forceFeeding = t.generateFakeWaypoints(5);
	std::vector<ros::Time> sendingTimes = t.sendGCSCommands(forceFeeding, gcs);

	ros::Duration(t.sleep_time).sleep();	//enough time to publish everything
	
	gcs.publish(t.regCom);			//goal_wp back to 0|0|0 for next test
	fprintf(stderr, "STOP STOP!\n");	
	gcs.publish(t.stop); //publish stop... transition back
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	

	//Now, seeing if the bins are equal...

	fprintf(stderr, "Trying to get received...\n");	
	std::vector<au_uav_ros::Command> ca_commands_bin = t.getReceivedCommands();	
	std::vector<ros::Time> time_bin = t.getReceivedTimes();		


	fprintf(stderr, "sent :%d | recv: %d", forceFeeding.size(), ca_commands_bin.size());	
	ASSERT_TRUE(forceFeeding.size() == ca_commands_bin.size()) << "sent != recv: " << forceFeeding.size() << " " << ca_commands_bin.size();
	for(int i =0; i< forceFeeding.size(); i++)	{
		fprintf(stderr, "received :: back lat: %f \n", ca_commands_bin[i].latitude);
		EXPECT_TRUE(forceFeeding[i].latitude == ca_commands_bin[i].latitude); 
		EXPECT_TRUE(forceFeeding[i].longitude == ca_commands_bin[i].longitude);
		EXPECT_TRUE(forceFeeding[i].altitude == ca_commands_bin[i].altitude) << "differ: " << forceFeeding[i] << "and " << ca_commands_bin[i]; 

		fprintf(stderr, " Time diff (s) command %d: %f\n\n", i, (time_bin[i] - sendingTimes[i]).toSec());	
	}		
}


TEST(mover, gradient_telem_feeding)	{


	test_state_machine t;
	ros::NodeHandle n;	
	int numWaypoints = 5;
	ros::Publisher gcs = n.advertise<au_uav_ros::Command>("gcs_commands", 5);	
	ros::Subscriber ca = n.subscribe("ca_commands", 10, &test_state_machine::ca_command_callback, &t);	
	

	//Transitioning to ca_on state	
	fprintf(stderr, "-> GRADIENT_telem\n");
	gcs.publish(t.stop);
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	
	gcs.publish(t.gradient_telem); //transition
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_chatty());	

	//Publishing some GCS commands
	std::vector<au_uav_ros::Command> forceFeeding = t.generateFakeWaypoints(5);
	std::vector<ros::Time> sendingTimes = t.sendGCSCommands(forceFeeding, gcs);

	ros::Duration(t.sleep_time).sleep();	//enough time to publish everything
	
	gcs.publish(t.regCom);			//goal_wp back to 0|0|0 for next test
	fprintf(stderr, "STOP STOP!\n");	
	gcs.publish(t.stop); //publish stop... transition back
	ros::Duration(t.sleep_time).sleep();	//time enough to measure lag initially
	EXPECT_TRUE(t.ca_commands_silent());	

	//Now, seeing if the bins are equal...

	fprintf(stderr, "Trying to get received...\n");	
	std::vector<au_uav_ros::Command> ca_commands_bin = t.getReceivedCommands();	
	std::vector<ros::Time> time_bin = t.getReceivedTimes();		


	fprintf(stderr, "sent :%d | recv: %d", forceFeeding.size(), ca_commands_bin.size());	
	ASSERT_TRUE(forceFeeding.size() == ca_commands_bin.size()) << "sent != recv: " << forceFeeding.size() << " " << ca_commands_bin.size();
	for(int i =0; i< forceFeeding.size(); i++)	{
		fprintf(stderr, "received :: back lat: %f \n", ca_commands_bin[i].latitude);
		EXPECT_TRUE(forceFeeding[i].latitude == ca_commands_bin[i].latitude); 
		EXPECT_TRUE(forceFeeding[i].longitude == ca_commands_bin[i].longitude);
		EXPECT_TRUE(forceFeeding[i].altitude == ca_commands_bin[i].altitude) << "differ: " << forceFeeding[i] << "and " << ca_commands_bin[i]; 

		fprintf(stderr, " Time diff (s) command %d: %f\n\n", i, (time_bin[i] - sendingTimes[i]).toSec());	
	}		
	
}	


int main(int argc, char ** argv)	{
	ros::init(argc, argv, "planeIDTester");
	testing::InitGoogleTest(&argc, argv);
	boost::thread spinner(spinThread); 	//calls ros::spin()
	boost::thread telem(pubTelem);		//continually publishes fake telemetry at 1hz, to force ca.avoid to run
	ros::Time::init();
	RUN_ALL_TESTS();
	fprintf(stderr, "test::TRYING TO SHTUDOWN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	ros::shutdown();
	spinner.join();
	telem.join();
}

