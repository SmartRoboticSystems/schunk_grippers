/*********************************************************************************************//**
* @file ezn64_usb_control_lib.h
*
* Header of ezn64_usb for USB communication between ROS and PG70 gripper
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
#ifndef EZN64_USB_CONTROL_H
#define EZN64_USB_CONTROL_H

#include <ros/ros.h>
#include <libusb.h>

#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//Service headers
#include <ezn64/reference.h>
#include <ezn64/set_position.h>
#include <ezn64/get_error.h>
#include <ezn64/get_position.h>
#include <ezn64/acknowledge_error.h>
#include <ezn64/stop.h>

class EZN64_usb
{
public:
    explicit EZN64_usb(ros::NodeHandle *nh);
    ~EZN64_usb();

     //Service callbacks
     bool referenceCallback(ezn64::reference::Request &req,
                            ezn64::reference::Response &res);

     bool setPositionCallback(ezn64::set_position::Request &req,
                              ezn64::set_position::Response &res);

     bool getErrorCallback(ezn64::get_error::Request &req,
                           ezn64::get_error::Response &res);

     bool getPositionCallback(ezn64::get_position::Request &req,
                              ezn64::get_position::Response &res);

     bool acknowledgeErrorCallback(ezn64::acknowledge_error::Request &req,
                                   ezn64::acknowledge_error::Response &res);

     bool stopCallback(ezn64::stop::Request &req,
                       ezn64::stop::Response &res);

     void timerCallback(const ros::TimerEvent &event);
     
     //EZN64 Joint state publisher
     ros::Publisher joint_pub;
     
private:
    
    //Gripper commands
    float setPosition(libusb_device_handle *handle, float goal_position);
    float getPosition(libusb_device_handle *handle);
    uint8_t getError(libusb_device_handle *handle);
    void stop(libusb_device_handle *handle);
    void acknowledgeError(libusb_device_handle *handle);
    void reference(libusb_device_handle *handle);
  
    //Libusb functions
    libusb_device* find_ezn64_dev(int VendorID, int ProductID);
    libusb_device_handle* open_ezn64_dev(libusb_device *dev);
    int close_ezn64_dev(libusb_device_handle *handle, libusb_context *usb_context);
    void usb_write(libusb_device_handle *handle, std::vector<uint8_t> output);
    std::vector<uint8_t> usb_read(libusb_device_handle *handle);
    void print_libusb_dev(libusb_device *dev);

    float IEEE_754_to_float(uint8_t *raw);
    void float_to_IEEE_754(float position, unsigned int *output_array);
    
    int gripper_id_;
    int vendor_id_;
    int product_id_;

    float act_position_;
    uint8_t ezn64_error_;
    sensor_msgs::JointState ezn64_joint_state_; 
    
    //LIBUSB Variables
    libusb_device *ezn64_dev_;
    libusb_context *usb_context_;
    libusb_device_handle *ezn64_handle_;

};  //EZN64_usb

#endif //EZN64_USB_CONTROL_H
