/*********************************************************************************************//**
* @file pg70_rs232_control.cpp
*
* Copyright (c)
* SmartRoboticSystems
* September 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/

/* Author: Frantisek Durovsky */

#include <pg70_rs232_control.h>

int 
main(int argc, char *argv[])
{
  ros::init(argc, argv, "pg70_rs232_control");
  ros::NodeHandle nh;

  //Create gripper object instance
  schunk_pg70::PG70_serial gripper(&nh);
     
  //Initialize user interface
  ros::ServiceServer reference_service         = nh.advertiseService("schunk_pg70/reference", &schunk_pg70::PG70_serial::referenceCallback, &gripper);
  ros::ServiceServer set_position_service      = nh.advertiseService("schunk_pg70/set_position", &schunk_pg70::PG70_serial::setPositionCallback, &gripper);
  ros::ServiceServer get_error_service         = nh.advertiseService("schunk_pg70/get_error", &schunk_pg70::PG70_serial::getErrorCallback, &gripper);
  ros::ServiceServer get_position_service      = nh.advertiseService("schunk_pg70/get_position", &schunk_pg70::PG70_serial::getPositionCallback, &gripper);
  ros::ServiceServer acknowledge_error_service = nh.advertiseService("schunk_pg70/acknowledge_error", &schunk_pg70::PG70_serial::acknowledgeErrorCallback, &gripper);
  ros::ServiceServer stop_service              = nh.advertiseService("schunk_pg70/stop", &schunk_pg70::PG70_serial::stopCallback, &gripper);
  
  ros::Timer timer = nh.createTimer(ros::Duration(gripper.TF_UPDATE_PERIOD), &schunk_pg70::PG70_serial::timerCallback, &gripper);
  gripper.joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);  
  
  ros::spin();

  return(EXIT_SUCCESS);
}
