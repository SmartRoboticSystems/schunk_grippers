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

//Service headers
#include <pg70/reference.h>
#include <pg70/set_position.h>
#include <pg70/get_state.h>
#include <pg70/acknowledge_error.h>
#include <pg70/stop.h>


class PG70_serial
{
public:
    explicit PG70_serial(ros::NodeHandle *nh);
    ~PG70_serial();
     uint16_t CRC16(uint16_t crc, uint16_t data);   //checksum function

     bool reference(serial::Serial *port);
     void set_position(serial::Serial *port, int position, int velocity, int acceleration);
     int  get_state(serial::Serial *port);
     bool stop(serial::Serial *port);
     bool acknowledge_error(serial::Serial *port);

     //Service callbacks
     bool reference_callback(pg70::reference::Request &req,
                             pg70::reference::Response &res);

     bool set_position_callback(pg70::set_position::Request &req,
                                pg70::set_position::Response &res);

     bool get_state_callback(pg70::get_state::Request &req,
                             pg70::get_state::Response &res);

     bool acknowledge_error_callback(pg70::acknowledge_error::Request &req,
                                     pg70::acknowledge_error::Response &res);

     bool stop_callback(pg70::stop::Request &req,
                        pg70::stop::Response &res);



private:
    serial::Serial *com_port;
    int gripper_id;
    std::string portname;
    double baudrate;
    uint32_t *ieee754_position_tbl;
    uint32_t *ieee754_velocity_tbl;
    uint32_t *ieee754_acceleration_tbl;



};

#endif //PG70_RS232_CONTROL_H
