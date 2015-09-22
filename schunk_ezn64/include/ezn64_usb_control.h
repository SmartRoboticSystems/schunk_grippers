/*********************************************************************************************//**
* @file ezn64_usb_control_lib.h
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

#ifndef EZN64_USB_CONTROL_H
#define EZN64_USB_CONTROL_H

#include <ros/ros.h>
#include <libusb-1.0/libusb.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

///Service headers
#include <schunk_ezn64/reference.h>
#include <schunk_ezn64/set_position.h>
#include <schunk_ezn64/get_error.h>
#include <schunk_ezn64/get_position.h>
#include <schunk_ezn64/acknowledge_error.h>
#include <schunk_ezn64/stop.h>

/** \brief Control of Schunk EZN64 gripper */
namespace schunk_ezn64
{

/** \brief Client class for EZN64 USB control */  
class EZN64_usb
{
public:
  
  /** \brief Construct a client for EZN64 USB control*/  
  EZN64_usb(ros::NodeHandle *nh);
  
  ~EZN64_usb();
  
  /** \brief Reference service callback */
  bool referenceCallback(schunk_ezn64::reference::Request &req,
                         schunk_ezn64::reference::Response &res);
     
  /** \brief GetError service callback */
  bool getErrorCallback(schunk_ezn64::get_error::Request &req,
                        schunk_ezn64::get_error::Response &res);
  
   /** \brief AcknowledgeError service callback */
  bool acknowledgeErrorCallback(schunk_ezn64::acknowledge_error::Request &req,
                                schunk_ezn64::acknowledge_error::Response &res);
  
  /** \brief SetPosition service callback */
  bool setPositionCallback(schunk_ezn64::set_position::Request &req,
                           schunk_ezn64::set_position::Response &res);
  
  /** \brief GetPosition service callback */
  bool getPositionCallback(schunk_ezn64::get_position::Request &req,
                           schunk_ezn64::get_position::Response &res);
  
   /** \brief Stop service callback */
  bool stopCallback(schunk_ezn64::stop::Request &req,
                    schunk_ezn64::stop::Response &res);
  
   /** \brief Timer callback to read USB input buffer periodically */
  void timerCallback(const ros::TimerEvent &event);  
  
   /** \brief Gripper joint state publisher */
  ros::Publisher joint_pub;
  
    /** \brief TF update period in seconds */
  static const float TF_UPDATE_PERIOD = 0.1;   
  
private:
    
  /** \brief Send CMD REFERENCE(0x92) command to the gripper */
  void reference(libusb_device_handle *handle);
   
  /** \brief Read actual error by GET STATE(0x95) command */
  uint8_t getError(libusb_device_handle *handle);
    
  /** \brief Send CMD_ACK(0x8b) command to the gripper */
  void acknowledgeError(libusb_device_handle *handle);
  
  /** \brief Read actual position by GET_STATE(0x95) command */
  float getPosition(libusb_device_handle *handle);
  
  /** \brief Send MOV_POS(0x80) command to the gripper */
  void setPosition(libusb_device_handle *handle, float goal_position);
    
  /** \brief Send CMD_STOP(0x91) to stop moving gripper */
  void stop(libusb_device_handle *handle);
   
  /** \brief Set periodic position reading by GET_STATE(0x95) command */
  void getPeriodicPositionUpdate(libusb_device_handle *handle, float period);
  
  /** \brief Function looking for gripper controller in list of available 
      USB devices. In case of success, pointer to device is returned */
  libusb_device* find_ezn64_dev(int VendorID, int ProductID);
  
  /** \brief Funtion printing information about active gripper USB interface */
  void print_libusb_dev(libusb_device *dev);

  /** \brief Function openning communication with USB device */
  libusb_device_handle* open_ezn64_dev(libusb_device *dev);
  
  /** \brief Function closing communication with USB device */
  int close_ezn64_dev(libusb_device_handle *handle, libusb_context *usb_context);

  /** \brief Function writing data to USB output buffer */
  void usb_write(libusb_device_handle *handle, std::vector<uint8_t> output);
  
  /** \brief Function reading data from USB inpput buffer*/
  std::vector<uint8_t> usb_read(libusb_device_handle *handle);
  
  /** \brief Conversion from 4 bytes to float*/
  float IEEE_754_to_float(uint8_t *raw);
  
  /** \brief Conversion from float to 4bytes*/
  void float_to_IEEE_754(float position, unsigned int *output_array);
  
  //Launch params
  int gripper_id_;
  int vendor_id_;
  int product_id_;
  double update_frequency;

  //Gripper state variales
  float act_position_;
  uint8_t ezn64_error_;
  sensor_msgs::JointState ezn64_joint_state_; 
    
  //Libusb Variables
  libusb_device *ezn64_dev_;
  libusb_context *usb_context_;
  libusb_device_handle *ezn64_handle_;
  
  //Consts 
  static const double MIN_GRIPPER_POS_LIMIT = 0;
  static const double MAX_GRIPPER_POS_LIMIT = 12;
  static const double WAIT_FOR_RESPONSE_INTERVAL = 0.5;
  static const int    INPUT_BUFFER_SIZE = 512;
  static const int    LIBUSB_ENDPOINT = 129;
  static const int    LIBUSB_TIMEOUT = 1000;
  static const int    LIBUSB_VERBOSITY_LEVEL = 3;
  static const int    URDF_SCALE_FACTOR = 1000;
  
};  //EZN64_usb
}   //schunk_ezn64

#endif //EZN64_USB_CONTROL_H
