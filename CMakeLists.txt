cmake_minimum_required(VERSION 2.8.3)
project(correction)

find_package(lcm REQUIRED)

find_package(catkin REQUIRED 
             cmake_modules
             COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             pcl_conversions
	     lcm_to_ros
	     roscpp
	     std_msgs
             roslib)

find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(PCL 1.7 REQUIRED)

add_definitions(-std=c++11)

include_directories(include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIR})

include_directories(
		    ${lcm_to_ros_SOURCE_DIR})


catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS} roslib DEPENDS)

add_executable(correction src/correction.cpp)
target_link_libraries(correction
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
		      ${lcm_to_ros_LIBRARIES}
			lcm) 
