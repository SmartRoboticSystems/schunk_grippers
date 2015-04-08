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
#include <serial/serial.h>
#include <libusb.h>

//Service headers
#include <ezn64/reference.h>
#include <ezn64/set_position.h>
#include <ezn64/get_state.h>
#include <ezn64/acknowledge_error.h>
#include <ezn64/stop.h>


class EZN64_usb
{
public:
    explicit EZN64_usb(ros::NodeHandle *nh);
    ~EZN64_usb();
     uint16_t CRC16(uint16_t crc, uint16_t data);   //checksum function

     int reference(libusb_device_handle *handle);
     int set_position(libusb_device_handle *handle, int position, int velocity, int acceleration);
     int  get_state(libusb_device_handle *handle);
     int stop(libusb_device_handle *handle);
     int acknowledge_error(libusb_device_handle *handle);

     //Service callbacks
     bool reference_callback(ezn64::reference::Request &req,
                             ezn64::reference::Response &res);

     bool set_position_callback(ezn64::set_position::Request &req,
                                ezn64::set_position::Response &res);

     bool get_state_callback(ezn64::get_state::Request &req,
                             ezn64::get_state::Response &res);

     bool acknowledge_error_callback(ezn64::acknowledge_error::Request &req,
                                     ezn64::acknowledge_error::Response &res);

     bool stop_callback(ezn64::stop::Request &req,
                        ezn64::stop::Response &res);

     //Libusb functions
     libusb_device* find_ezn64_dev(int VendorID, int ProductID);
     libusb_device_handle* open_ezn64_dev(libusb_device *dev);
     int close_ezn64_dev(libusb_device_handle *handle, libusb_context *usb_context);
     int usb_transaction(libusb_device_handle *handle, std::vector<uint8_t> output);



     void print_libusb_dev(libusb_device *dev);

private:
    int gripper_id;
    int vendor_id;
    int product_id;

    uint32_t *ieee754_position_tbl;
    uint32_t *ieee754_velocity_tbl;
    uint32_t *ieee754_acceleration_tbl;

    //LIBUSB Variables
    libusb_device *ezn64_dev;
    libusb_context *usb_context;
    libusb_device_handle *ezn64_handle;






};

#endif //EZN64_USB_CONTROL_H
