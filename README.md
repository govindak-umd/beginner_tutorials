[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# ROS Publisher/Subscriber

## Overview

This code is a demo of ROS Service.

A few of the major files in this package is :

talker.cpp (Publisher)
listener.cpp (Subscriber)
nodes_launch.launch (Launch file)
changeStringService.srv (Service File)

## Assumptions
		ROS Melodic
		Ubuntu 18.04
## General structure of a catkin workspace

		workspace_folder/        -- WORKSPACE
		  src/                   -- SOURCE SPACE
		    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
		    package_1/
		      CMakeLists.txt     -- CMakeLists.txt file for package_1
		      package.xml        -- Package manifest for package_1
		    ...
		    package_n/
		      CMakeLists.txt     -- CMakeLists.txt file for package_n
		      package.xml        -- Package manifest for package_n

## Directory structure of this catkin_ws

		catkin_ws/
			build
			devel
			src
				beginner_tutorials
					CMakeLists.txt
					package.xml
				CMakeLists.txt

## To create the catkin package, building and sourcing it

		$ cd ~/catkin_ws/src
		$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
		$ cd ..
		$ catkin_make
		$ . ~/catkin_ws/devel/setup.bash

## Creating the publisher and subscribers

		$ roscd beginner_tutorials
		$ mkdir -p src

## Modifying the CMakeLists.txt

		cmake_minimum_required(VERSION 2.8.3)
		project(beginner_tutorials)

		## Find catkin and any catkin packages
		find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

		## Declare ROS messages and services
		#add_message_files(DIRECTORY msg FILES Num.msg)
		#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

		## Generate added messages and services
		generate_messages(DEPENDENCIES std_msgs)

		## Declare a catkin package
		catkin_package()

		add_executable(talker src/talker.cpp)
		target_link_libraries(talker ${catkin_LIBRARIES})
		add_dependencies(talker beginner_tutorials_generate_messages_cpp)

		add_executable(listener src/listener.cpp)
		target_link_libraries(listener ${catkin_LIBRARIES})
		add_dependencies(listener beginner_tutorials_generate_messages_cpp)

## Running catkin_make

		$ cd ~/catkin_ws
		$ catkin_make  

## Before running the talker

		$ roscore

## Running the publisher in a new terminal 

		$ cd ~/catkin_ws
		$ source ./devel/setup.bash
		$ rosrun beginner_tutorials talker      (C++)

## Running the subscriber in a new terminal 

		$ cd ~/catkin_ws
		$ source ./devel/setup.bash
		$ rosrun beginner_tutorials listener     (C++)

## To launch the launch file (add the custom frequency at the end) 

		$ roslaunch beginner_tutorials nodes_launch.launch freq:=5

## To call the ROS Service
		$ rosservice call /changeStringService "String to change the ROS SERVICE"

## Logging using the rqt_console
		$ rqt_console	

## To terminate

		press Ctrl-C to terminate both the listener and the talker

## Running cpp-check

		cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./src/")

## Running cpp-lint

		cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )

