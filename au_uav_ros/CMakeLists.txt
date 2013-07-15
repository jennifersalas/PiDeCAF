cmake_minimum_required(VERSION 2.8.3)
project(au_uav_ros)
find_package(catkin REQUIRED
  COMPONENTS genmsg message_generation std_msgs roscpp rospy roslib rostest
)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(FILES
  Command.msg
  Telemetry.msg
)

add_service_files(FILES
	planeIDGetter.srv
)

SET(
  CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#no glib
#find_package(GLIB2 REQUIRED)
#find_Package(GTHREAD2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread) #do i need locks? I'm so confused it seems to be workign without this

INCLUDE_DIRECTORIES(
  ${MAVLINK_INCLUDE_DIR}
#  ${GLIB2_MAIN_INCLUDE_DIR}
#  ${GLIB2_INTERNAL_INCLUDE_DIR}
#  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ../include/
)

generate_messages(DEPENDENCIES
  std_msgs
)

#add_library(standardFuncs src/standardFuncs.cpp include/au_uav_ros/standardFuncs.h)
#add_library(standardDefs src/standardDefs.cpp include/au_uav_ros/standardDefs.h)
#add_library(planeBuilder src/planeBuilder.cpp include/au_uav_ros/planeBuilder.h src/standardDefs.cpp include/au_uav_ros/standardDefs.h)
#add_library(planeBuilder src/planeBuilder.cpp include/au_uav_ros/planeBuilder.h)
add_library(planeObject src/planeObject.cpp  src/standardFuncs.cpp  src/standardDefs.cpp )
#add_library(planeObject src/planeObject.cpp include/au_uav_ros/planeObject.h)
#add_library(simPlaneObject src/simPlaneObject.cpp include/au_uav_ros/simPlaneObject.h)
#add_library(vmath src/vmath.cpp include/au_uav_ros/vmath.h)
#add_library(ripna src/ripna.cpp include/au_uav_ros/ripna.h)
#add_library(collisionAvoidance src/collisionAvoidance.cpp include/au_uav_ros/collisionAvoidance.h)
add_library(serial_talker src/serial_talker.cpp)
add_library(mavlink_fun src/mavlink_read.cpp)
add_library(collision_avoidance src/collision_avoidance.cpp)

add_library(fsquared src/planeObject.cpp src/Fsquared.cpp src/vmath.cpp src/ForceField.cpp)


#add_executable(xbee src/test.cpp)

#Xbee talker
add_executable(xbee src/xbee_talker.cpp)
add_dependencies(xbee ${PROJECT_NAME}_gencpp)
target_link_libraries(xbee serial_talker mavlink_fun)

#ardu talker
add_executable(ardu src/ardu_talker.cpp)
add_dependencies(ardu ${PROJECT_NAME}_gencpp)
target_link_libraries(ardu serial_talker mavlink_fun)

#GCS talker
add_executable(gcs src/gcs_talker.cpp)
add_dependencies(gcs ${PROJECT_NAME}_gencpp)
target_link_libraries(gcs serial_talker mavlink_fun)

#mover
add_executable(mover src/mover.cpp)
add_dependencies(mover ${PROJECT_NAME}_gencpp)
target_link_libraries(mover planeObject collision_avoidance fsquared)


#collision avoidance logic
#add_executable(ca_logic src/collision_avoidance_logic.cpp)
#add_dependencies(ca_logic ${PROJECT_NAME}_gencpp)
#target_link_libraries(ca_logic mavlink_fun fsquared)


#add_dependencies(standardDefs ${PROJECT_NAME}_gencpp)
#add_dependencies(standardFuncs ${PROJECT_NAME}_gencpp)
#add_dependencies(simulator ${PROJECT_NAME}_gencpp)
#add_dependencies(simPlaneObject ${PROJECT_NAME}_gencpp)
#add_dependencies(xbeeIn ${PROJECT_NAME}_gencpp)
#add_dependencies(xbeeOut ${PROJECT_NAME}_gencpp)
#add_dependencies(guiInterfacer ${PROJECT_NAME}_gencpp)
#add_dependencies(coordinator ${PROJECT_NAME}_gencpp)
#add_dependencies(planeBuilder ${PROJECT_NAME}_gencpp)

#target_link_libraries(xbeeIn
#  ${GLIB2_LIBRARY}
#  ${GTHREAD2_LIBRARY}
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(xbeeOut
#  ${GLIB2_LIBRARY}
#  ${GTHREAD2_LIBRARY}
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(
#  ${PROJECT_NAME}
#  ${BOOST_LIBRARIES}
#)

target_link_libraries(xbee ${catkin_LIBRARIES})
target_link_libraries(ardu ${catkin_LIBRARIES})
target_link_libraries(gcs ${catkin_LIBRARIES})
target_link_libraries(mover ${catkin_LIBRARIES})
#target_link_libraries(ca_logic ${catkin_LIBRARIES})
#target_link_libraries(ripna vmath)
#target_link_libraries(standardDefs ${BOOST_LIBRARIES})
#target_link_libraries(planeBuilder standardDefs)
#target_link_libraries(planeObject standardFuncs standardDefs)
#target_link_libraries(collisionAvoidance ripna)
#target_link_libraries(simPlaneObject planeObject)
#target_link_libraries(simulator ${catkin_LIBRARIES} simPlaneObject)
#target_link_libraries(guiInterfacer ${catkin_LIBRARIES})
#target_link_libraries(coordinator ${catkin_LIBRARIES} planeBuilder simPlaneObject planeObject collisionAvoidance vmath)

#Unit testing (empty for now)
#catkin_add_gtest(collisionAvoidance  test/ca_tester.cpp)

#Node Testing
catkin_add_gtest(planeIDServer_tester src/test/serverSideIDTester.cpp)
target_link_libraries(planeIDServer_tester ${catkin_LIBRARIES})
set_target_properties(planeIDServer_tester PROPERTIES OUTPUT_NAME planeIDServer_tester)
add_rostest(test/planeIDServerTester.test)

catkin_add_gtest(mover_state_tester src/test/moverStateMachine.cpp)
target_link_libraries(mover_state_tester ${catkin_LIBRARIES})
set_target_properties(mover_state_tester PROPERTIES OUTPUT_NAME mover_state_tester)
add_rostest(test/mover_state_tester.test)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp AU_UAV_GUI message_runtime
)
