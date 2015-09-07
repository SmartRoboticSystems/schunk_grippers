/*********************************************************************************************//**
* @file pg70_rs232_control_lib.h
*
* Header of pg70_serial for serial communication between ROS and PG70 gripper
*
* Copyright (c)
* Frantisek Durovsky
* Department of Robotics
* Technical University Kosice
* April 2015
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
#ifndef PG70_RS232_CONTROL_H
#define PG70_RS232_CONTROL_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//Service headers
#include <pg70/reference.h>
#include <pg70/set_position.h>
#include <pg70/get_error.h>
#include <pg70/get_position.h>
#include <pg70/acknowledge_error.h>
#include <pg70/stop.h>

class PG70_serial
{
public:
    explicit PG70_serial(ros::NodeHandle *nh);
    ~PG70_serial();
  
    //Service callbacks
    bool referenceCallback(pg70::reference::Request &req,
                           pg70::reference::Response &res);

    bool setPositionCallback(pg70::set_position::Request &req,
                             pg70::set_position::Response &res);

    bool getErrorCallback(pg70::get_error::Request &req,
                          pg70::get_error::Response &res);

    bool getPositionCallback(pg70::get_position::Request &req,
                             pg70::get_position::Response &res);

    bool acknowledgeErrorCallback(pg70::acknowledge_error::Request &req,
                                   pg70::acknowledge_error::Response &res);

    bool stopCallback(pg70::stop::Request &req,
                      pg70::stop::Response &res);

    void timerCallback(const ros::TimerEvent &event);
        
    //PG70 Joint state publisher
    ros::Publisher joint_pub;

private:
     
   //Gripper commands
    float setPosition(serial::Serial *port, int goal_position, int velocity, int acceleration);
    float getPosition(serial::Serial *port);
    uint8_t getError(serial::Serial *port);
    void stop(serial::Serial *port);
    void acknowledgeError(serial::Serial *port);
    void reference(serial::Serial *port);
  
    //Checksum function
    uint16_t CRC16(uint16_t crc, uint16_t data);   
    
    //Float - IEEE754 conversions
    float IEEE_754_to_float(uint8_t *raw);
    void float_to_IEEE_754(float position, unsigned int *output_array);
    
    //PG70 variables
    sensor_msgs::JointState pg70_joint_state_; 
    serial::Serial *com_port_;
    int gripper_id_;
    std::string port_name_;
    double baudrate_;

    float act_position_;
    uint8_t pg70_error_;

};  //PG70_serial

#endif //PG70_RS232_CONTROL_H
