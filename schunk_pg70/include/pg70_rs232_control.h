/*********************************************************************************************//**
* @file pg70_rs232_control_lib.h
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

#ifndef PG70_RS232_CONTROL_H
#define PG70_RS232_CONTROL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//Service headers
#include <schunk_pg70/reference.h>
#include <schunk_pg70/set_position.h>
#include <schunk_pg70/get_error.h>
#include <schunk_pg70/get_position.h>
#include <schunk_pg70/acknowledge_error.h>
#include <schunk_pg70/stop.h>

/** \brief Control of Schunk PG70 gripper */
namespace schunk_pg70
{

/** \brief Client class for PG70 serial control */  
class PG70_serial
{
public:
  
  /** \brief Construct a client for PG70 serial control*/    
  PG70_serial(ros::NodeHandle *nh);
  
  ~PG70_serial();
  
  /** \brief Reference service callback */
  bool referenceCallback(schunk_pg70::reference::Request &req,
                         schunk_pg70::reference::Response &res);
  
  /** \brief GetError service callback */
  bool getErrorCallback(schunk_pg70::get_error::Request &req,
                        schunk_pg70::get_error::Response &res);
  
  /** \brief GetPosition service callback */
  bool getPositionCallback(schunk_pg70::get_position::Request &req,
                           schunk_pg70::get_position::Response &res);

   /** \brief SetPosition service callback */
  bool setPositionCallback(schunk_pg70::set_position::Request &req,
                           schunk_pg70::set_position::Response &res);
  
   /** \brief AcknowledgeError service callback */
  bool acknowledgeErrorCallback(schunk_pg70::acknowledge_error::Request &req,
                                schunk_pg70::acknowledge_error::Response &res);
  
  /** \brief Stop service callback */
  bool stopCallback(schunk_pg70::stop::Request &req,
                    schunk_pg70::stop::Response &res);
  
   /** \brief Timer callback to read serial input buffer periodically */
  void timerCallback(const ros::TimerEvent &event);  
  
  /** \brief Gripper joint state publisher */
  ros::Publisher joint_pub;
  
  /** \brief TF update period in seconds */
  static const float TF_UPDATE_PERIOD = 0.1;   
   
private:

  /** \brief Send CMD REFERENCE(0x92) command to the gripper */
  void reference(serial::Serial *port);
   
  /** \brief Read actual error by GET STATE(0x95) command */
  uint8_t getError(serial::Serial *port);
    
  /** \brief Read actual position by GET_STATE(0x95) command */
  float getPosition(serial::Serial *port);
  
  /** \brief Send MOV_POS(0x80) command to the gripper */
  void setPosition(serial::Serial *port, int goal_position, int velocity, int acceleration);
   
  /** \brief Send CMD_ACK(0x8b) command to the gripper */
  void acknowledgeError(serial::Serial *port);
    
  /** \brief Send CMD_STOP(0x91) to stop moving gripper */
  void stop(serial::Serial *port);
    
  /** \brief Set periodic position reading by GET_STATE(0x95) command */
  void getPeriodicPositionUpdate(serial::Serial *port, float update_frequency);
  
  /** \brief Function to determine checksum*/
  uint16_t CRC16(uint16_t crc, uint16_t data);   
     
  /** \brief Conversion from 4 bytes to float*/
  float IEEE_754_to_float(uint8_t *raw);
  
  /** \brief Conversion from float to 4 bytes*/
  void float_to_IEEE_754(float position, unsigned int *output_array);
   
  //Launch params
  int gripper_id_;
  std::string port_name_;
  int baudrate_;  

  //Gripper state variables
  float act_position_;
  uint8_t pg70_error_;
  sensor_msgs::JointState pg70_joint_state_; 

  //Serial variables
  serial::Serial *com_port_;
  
  //Consts
  static const double MIN_GRIPPER_POS_LIMIT = 0;
  static const double MAX_GRIPPER_POS_LIMIT = 69;
  static const double MIN_GRIPPER_VEL_LIMIT = 0;
  static const double MAX_GRIPPER_VEL_LIMIT = 83;
  static const double MIN_GRIPPER_ACC_LIMIT = 0;
  static const double MAX_GRIPPER_ACC_LIMIT = 320;
  static const double WAIT_FOR_RESPONSE_INTERVAL = 0.5;
  static const double INPUT_BUFFER_SIZE = 64;
  static const int    URDF_SCALE_FACTOR = 2000;
    
};  //PG70_serial
}   //schunk_pg70

#endif //PG70_RS232_CONTROL_H
