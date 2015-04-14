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
#include <pg70/get_error.h>
#include <pg70/get_position.h>
#include <pg70/acknowledge_error.h>
#include <pg70/stop.h>


class PG70_serial
{
public:
    explicit PG70_serial(ros::NodeHandle *nh);
    ~PG70_serial();
    uint16_t CRC16(uint16_t crc, uint16_t data);   //checksum function

    //Gripper commands
    float set_position(serial::Serial *port, int goal_position, int velocity, int acceleration);
    float get_position(serial::Serial *port);
    uint8_t get_error(serial::Serial *port);
    void stop(serial::Serial *port);
    void acknowledge_error(serial::Serial *port);
    void reference(serial::Serial *port);

    //Service callbacks
    bool reference_callback(pg70::reference::Request &req,
                            pg70::reference::Response &res);

    bool set_position_callback(pg70::set_position::Request &req,
                               pg70::set_position::Response &res);

    bool get_error_callback(pg70::get_error::Request &req,
                            pg70::get_error::Response &res);

    bool get_position_callback(pg70::get_position::Request &req,
                               pg70::get_position::Response &res);

    bool acknowledge_error_callback(pg70::acknowledge_error::Request &req,
                                    pg70::acknowledge_error::Response &res);

    bool stop_callback(pg70::stop::Request &req,
                       pg70::stop::Response &res);

    //Float - IEEE754 conversions
    float IEEE_754_to_float(uint8_t *raw);
    void float_to_IEEE_754(float position, unsigned int *output_array);

private:
    serial::Serial *com_port;
    int gripper_id;
    std::string portname;
    double baudrate;

    float act_position;
    uint8_t pg70_error;

};

#endif //PG70_RS232_CONTROL_H
