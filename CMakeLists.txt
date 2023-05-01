cmake_minimum_required(VERSION 2.8.3)
project(common_nodes)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wno-psabi ${CMAKE_CXX_FLAGS}")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    mavros
    message_generation
    geometry_msgs
    sensor_msgs
    std_msgs
    roscpp
    tf
)

find_package(Boost REQUIRED COMPONENTS thread)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

#generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   geometry_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flight_pkg
   CATKIN_DEPENDS 
   mavros
   geometry_msgs
   nav_msgs
   trajectory_msgs
   roscpp 
   sensor_msgs
   std_msgs
   roscpp
   tf
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include src ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(logger_node src/logger.cpp)
target_link_libraries(logger_node ${catkin_LIBRARIES})

add_executable(hybrid_logger_node src/hybrid.cpp)
target_link_libraries(hybrid_logger_node ${catkin_LIBRARIES})

add_executable(ball_logger_node src/ball_position_logger.cpp)
target_link_libraries(ball_logger_node ${catkin_LIBRARIES})

add_executable(m100_log_node src/matrice_logger.cpp)
target_link_libraries(m100_log_node ${catkin_LIBRARIES})

add_executable(m100_force_log_node src/force_control_logger.cpp)
target_link_libraries(m100_force_log_node ${catkin_LIBRARIES})





