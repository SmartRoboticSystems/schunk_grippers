/*********************************************************************************************//**
* @file ezn64_usb_control_lib.cpp
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

#ifndef EZN64_USB_CONTROL_LIB_CPP
#define EZN64_USB_CONTROL_LIB_CPP

#include <ezn64_usb_control.h>
namespace schunk_ezn64
{

EZN64_usb::EZN64_usb(ros::NodeHandle *nh) :
    act_position_(-1),
    ezn64_error_(0xff)
{
  //Read launch file params
  nh->getParam("schunk_ezn64/gripper_id", gripper_id_);
  nh->getParam("schunk_ezn64/vendor_id",  vendor_id_);
  nh->getParam("schunk_ezn64/product_id", product_id_);
     
  //Look for Schunk EZN64 controller on USB
  ezn64_dev_ = find_ezn64_dev(vendor_id_, product_id_);
  print_libusb_dev(ezn64_dev_);
  
  //If EZN64 found successfully, initialize interface
  if(ezn64_dev_ != 0)
  {   
    ezn64_handle_ = open_ezn64_dev(ezn64_dev_);
    
    //Get initial state and discard input buffer
    while (ezn64_error_ == 0xff)
    {
      ezn64_error_ = getError(ezn64_handle_);
      ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
    }
    
    //If emergency stop error occured, acknowledge
    if(ezn64_error_ == 0xd9)
    {
      ROS_INFO_STREAM("Acknowleding Emergency stop error ...");
      acknowledgeError(ezn64_handle_);
    }
  
    //Start periodic gripper position reading
    getPeriodicPositionUpdate(ezn64_handle_, TF_UPDATE_PERIOD);     
  } 
}

EZN64_usb::~EZN64_usb()
{
  close_ezn64_dev(ezn64_handle_, usb_context_);
}

void
EZN64_usb::reference(libusb_device_handle *handle)
{
  ROS_INFO("EZN64: Referencing");
  std::vector<uint8_t> output;
  output.push_back(0x01);           //Data Length
  output.push_back(0x92);           //Command reference

  //Send message to the gripper
  usb_write(handle, output);
}

uint8_t
EZN64_usb::getError(libusb_device_handle *handle)
{
  std::vector<uint8_t> output;
  output.push_back(0x06);                 //Data Length
  output.push_back(0x95);                 //Command get state
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x01);

  //Send get_state message to the module and recieve response
  usb_write(handle, output);

  ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
  std::vector<uint8_t> input;
  input = usb_read(handle);

  //Read and process current state message
  int get_state_index = 0;
  if(input.size() > 0)
  {
    if(input.size() > 7)
    {
      for(size_t i = 0; i < input.size(); i++)
        if ((input[i] == 0x07) && (input[i+1] == 0x95)) get_state_index = i;

      int ezn64_error_ = input[get_state_index + 7];

      switch (ezn64_error_)
      {
        case 0x00: ROS_INFO("EZN64: No error detected"); break;
        case 0xC8: ROS_ERROR("EZN64: Error: 0xC8 detected: Wrong ramp type"); break;
        case 0xD2: ROS_ERROR("EZN64: Error: 0xD2 detected: Config memory"); break;
        case 0xD3: ROS_ERROR("EZN64: Error: 0xD3 detected: Program memory"); break;
        case 0xD4: ROS_ERROR("EZN64: Error: 0xD4 detected: Invalid phrase"); break;
        case 0xD5: ROS_ERROR("EZN64: Error: 0xD5 detected: Soft low"); break;
        case 0xD6: ROS_ERROR("EZN64: Error: 0xD6 detected: Soft high"); break;
        case 0xD7: ROS_ERROR("EZN64: Error: 0xD7 detected: Pressure"); break;
        case 0xD8: ROS_ERROR("EZN64: Error: 0xD8 detected: Service required"); break;
        case 0xD9: ROS_ERROR("EZN64: Error: 0xD9 detected: Emergency stop"); break;
        case 0xDA: ROS_ERROR("EZN64: Error: 0xDA detected: Tow"); break;
        case 0xE4: ROS_ERROR("EZN64: Error: 0xE4 detected: Too fast"); break;
        case 0xEC: ROS_ERROR("EZN64: Error: 0xEC detected: Math error"); break;
        case 0xDB: ROS_ERROR("EZN64: Error: 0xDB detected: VPC3"); break;
        case 0xDC: ROS_ERROR("EZN64: Error: 0xDC detected: Fragmentation"); break;
        case 0xDE: ROS_ERROR("EZN64: Error: 0xDE detected: Current"); break;
        case 0xDF: ROS_ERROR("EZN64: Error: 0xDF detected: I2T"); break;
        case 0xE0: ROS_ERROR("EZN64: Error: 0xE0 detected: Initialize"); break;
        case 0xE1: ROS_ERROR("EZN64: Error: 0xE1 detected: Internal"); break;
        case 0xE2: ROS_ERROR("EZN64: Error: 0xE2 detected: Hard low"); break;
        case 0xE3: ROS_ERROR("EZN64: Error: 0xE3 detected: Hard high"); break;
        case 0x70: ROS_ERROR("EZN64: Error: 0x70 detected: Temp low"); break;
        case 0x71: ROS_ERROR("EZN64: Error: 0x71 detected: Temp high"); break;
        case 0x72: ROS_ERROR("EZN64: Error: 0x72 detected: Logic low"); break;
        case 0x73: ROS_ERROR("EZN64: Error: 0x73 detected: Logic high"); break;
        case 0x74: ROS_ERROR("EZN64: Error: 0x74 detected: Motor voltage low"); break;
        case 0x75: ROS_ERROR("EZN64: Error: 0x75 detected: Motor voltage high"); break;
        case 0x76: ROS_ERROR("EZN64: Error: 0x76 detected: Cable break"); break;
        case 0x78: ROS_ERROR("EZN64: Error: 0x78 detected: Motor temp "); break;
      }
      //Reset periodic position reading
       getPeriodicPositionUpdate(handle, TF_UPDATE_PERIOD);
   
       return((uint8_t)ezn64_error_);
    }
    else
    {
      ROS_WARN("EZN64: Not get_state response");
            
      //Reset periodic position reading
      getPeriodicPositionUpdate(handle, TF_UPDATE_PERIOD);
      return(0xff) ;
    }
  }
  output.clear();
}

void
EZN64_usb::acknowledgeError(libusb_device_handle *handle)
{
  std::vector<uint8_t> output;
  output.push_back(0x01);      //Data Length
  output.push_back(0x8b);      //Command cmd ack

  //Send message to the module
  usb_write(handle, output);
}

void
EZN64_usb::setPosition(libusb_device_handle *handle, float goal_position)
{
  ROS_INFO_STREAM("EZN64: Moving from: " << act_position_ << " [mm] to " << goal_position << " [mm]");

  std::vector<uint8_t> output;
  output.push_back(0x05);                //Data Length
  output.push_back(0xb0);                //Command mov pos

  unsigned int IEEE754_bytes[4];
  float_to_IEEE_754(goal_position, IEEE754_bytes);

  output.push_back(IEEE754_bytes[0]);    //Position first byte
  output.push_back(IEEE754_bytes[1]);    //Position second byte
  output.push_back(IEEE754_bytes[2]);    //Position third byte
  output.push_back(IEEE754_bytes[3]);    //Position fourth byte

  //Send message to the module and recieve response
  usb_write(handle, output);       
}

float
EZN64_usb::getPosition(libusb_device_handle *handle)
{
  std::vector<uint8_t> output;
  output.push_back(0x06);                 //Data Length
  output.push_back(0x95);                 //Command get state
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x00);
  output.push_back(0x01);                //Only once

  //Send get_state message to the module and recieve response
  usb_write(handle, output);

  ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
  std::vector<uint8_t> input;
  input = usb_read(handle);

  float act_position;

  if(input.size() > 0)
  {
    for(size_t i = 0; i < input.size(); i++)
    {
      //Handle both possible response messages
      if((input[i] == 0x07) && (input[i+1] == 0x95))
      {
        uint8_t raw[4] = {input[i+5], input[i+4], input[i+3], input[i+2]};
        act_position = IEEE_754_to_float(raw);
        ROS_INFO_STREAM("EZN64 INFO: Actual position: " << act_position << "[mm]");
        return(act_position);
      }
      else if ((input[i] == 0x05) && (input[i+1] == 0x94))
      {
        uint8_t raw[4] = {input[i+5], input[i+4], input[i+3], input[i+2]};
        act_position = IEEE_754_to_float(raw);
        ROS_INFO_STREAM("EZN64 INFO: Actual position: " << act_position << "[mm]");
        return(act_position);
      }
    }
  }
}

void
EZN64_usb::getPeriodicPositionUpdate(libusb_device_handle *handle, float update_period)
{
  std::vector<uint8_t> output;
  output.push_back(0x06);                 //Data Length
  output.push_back(0x95);                 //Command get state
    
  unsigned int IEEE754_bytes[4];
  float_to_IEEE_754(update_period, IEEE754_bytes);
      
  output.push_back(IEEE754_bytes[0]);    //period first byte
  output.push_back(IEEE754_bytes[1]);    //period second byte
  output.push_back(IEEE754_bytes[2]);    //period third byte
  output.push_back(IEEE754_bytes[3]);    //period fourth byte
  output.push_back(0x07);

  //Send get_state message to the module 
  usb_write(handle, output);   
}

void
EZN64_usb::stop(libusb_device_handle *handle)
{
  std::vector<uint8_t> output;
  output.push_back(0x01);       //Data Length
  output.push_back(0x91);       //Command stop

  //Send message to the module
  usb_write(handle, output);
}

///////////////////////////////////////////////////////////////
//CALLBACKS
///////////////////////////////////////////////////////////////

bool
EZN64_usb::referenceCallback(schunk_ezn64::reference::Request &req,
                             schunk_ezn64::reference::Response &res)
{
  ROS_INFO("EZN64: Reference Cmd recieved ");
  reference(ezn64_handle_);           
}

bool
EZN64_usb::setPositionCallback(schunk_ezn64::set_position::Request &req,
                               schunk_ezn64::set_position::Response &res)
{
  //Check if goal request respects gripper limits <0-12> mm
  if ((req.goal_position >= MIN_GRIPPER_POS_LIMIT)
     && (req.goal_position <= MAX_GRIPPER_POS_LIMIT))
  {
    setPosition(ezn64_handle_,req.goal_position);  
    res.goal_accepted = true;
  }
  else
  {
    ROS_WARN("EZN64: Goal position rejected!");
    res.goal_accepted = false;
  }
}

bool
EZN64_usb::getErrorCallback(schunk_ezn64::get_error::Request &req,
                            schunk_ezn64::get_error::Response &res)
{
  ROS_INFO:("EZN64: Get Error request recieved");
  res.error_code = getError(ezn64_handle_);
}

bool
EZN64_usb::getPositionCallback(schunk_ezn64::get_position::Request &req,
                               schunk_ezn64::get_position::Response &res)
{
  ROS_INFO("EZN64: Get position request recieved");
  res.actual_position = act_position_;    
}

bool
EZN64_usb::acknowledgeErrorCallback(schunk_ezn64::acknowledge_error::Request &req,
                                    schunk_ezn64::acknowledge_error::Response &res)
{
  ROS_INFO("EZN64: Cmd Acknowledge error recieved");
  acknowledgeError(ezn64_handle_);
  ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
  if(getError(ezn64_handle_) == 0x00)
    res.acknowledge_response = true;
  else
    res.acknowledge_response = false;   
}


bool
EZN64_usb::stopCallback(schunk_ezn64::stop::Request &req,
                        schunk_ezn64::stop::Response &res)
{
  ROS_INFO("EZN64: Cmd Stop recieved");
  stop(ezn64_handle_);
  ros::Duration(WAIT_FOR_RESPONSE_INTERVAL).sleep();
  res.stop_result = getPosition(ezn64_handle_); 
}

void
EZN64_usb::timerCallback(const ros::TimerEvent &event)
{
  std::vector<uint8_t> input;
  input = usb_read(ezn64_handle_);
    
  //Detect position reached response
  for(size_t i = 0; i < input.size(); i++)
  {
    if((input[i] == 0x07) && (input[i+1] == 0x95))
    {
      uint8_t raw[4] = {input[i+5], input[i+4], input[i+3], input[i+2]};
      act_position_ = IEEE_754_to_float(raw);      
    }
    else if ((input[i] == 0x0f) && (input[i+2] == 0x95))
    {
      uint8_t raw[4] = {input[i+6], input[i+5], input[i+4], input[i+3]};
      act_position_ = IEEE_754_to_float(raw);      
    }
    else if ((input[i] == 0x05) && (input[i+1] == 0x94))
    {
      uint8_t raw[4] = {input[i+5], input[i+4], input[i+3], input[i+2]};
      act_position_ = IEEE_754_to_float(raw);      
    }
  }
  
  //Publish TF
  ezn64_joint_state_.header.stamp = ros::Time::now();
  ezn64_joint_state_.name.clear();
  ezn64_joint_state_.position.clear();
    
  ezn64_joint_state_.name.push_back("ezn64_finger_1_joint");
  ezn64_joint_state_.position.push_back(act_position_/URDF_SCALE_FACTOR);
  
  ezn64_joint_state_.name.push_back("ezn64_finger_2_joint");
  ezn64_joint_state_.position.push_back(act_position_/URDF_SCALE_FACTOR);
  
  ezn64_joint_state_.name.push_back("ezn64_finger_3_joint");
  ezn64_joint_state_.position.push_back(act_position_/URDF_SCALE_FACTOR);
   
  joint_pub.publish(ezn64_joint_state_);         
  
}

/////////////////////////////////////////////////////////
//LIBUSB FUNCTIONS
/////////////////////////////////////////////////////////

libusb_device*
EZN64_usb::find_ezn64_dev(int VendorID, int ProductID)
{
  int r;

  //Initialize USB session
  r= libusb_init(&usb_context_);
  if (r < 0)
  {
    ROS_ERROR("EZN64: Init Error");
    return 0;
  }
  libusb_set_debug(usb_context_, LIBUSB_VERBOSITY_LEVEL);        //set verbosity level to 3, as suggested in the documentation

  //Get list of devices
  static size_t dev_cnt;          //number of devices
  libusb_device **devs;           //pointer to pointer of device used to retrieve a list of devices
  dev_cnt = libusb_get_device_list(usb_context_, &devs);
  if(dev_cnt < 0)
  {
    ROS_ERROR("EZN64: Get device Error");
    return 0;
  }
  else
    ROS_INFO_STREAM("EZN64: Number of devices: " << dev_cnt);

  for(size_t i = 0; i < dev_cnt; i++)
  {
    struct libusb_device_descriptor descriptor;
    if(libusb_get_device_descriptor(devs[i], &descriptor))
    {
      ROS_ERROR("EZN64: Not able to read descriptor");
      return 0;
    }
  
    if((descriptor.idVendor == VendorID) && (descriptor.idProduct == ProductID))
    {
      ROS_INFO("EZN64: Gripper found");
      return devs[i];
    }
  }
  libusb_free_device_list(devs, 1 );     /* Free list and all not open devices */
}

libusb_device_handle*
EZN64_usb::open_ezn64_dev(libusb_device *dev)
{
  libusb_device_handle* handle;

  //Open device
  if(libusb_open(dev, &handle))
  {
    ROS_ERROR("EZN64: Port not oppened!");
    return 0;
  }
  else
  {
    ROS_INFO("EZN64: USB interface oppened successfully! ");
    return handle;
  }
}

int
EZN64_usb::close_ezn64_dev(libusb_device_handle *handle, libusb_context *usb_context_)
{
  ROS_INFO("EZN64: Closing USB port");
  libusb_close(handle);
  libusb_exit(usb_context_);
  return 0;
}

void
EZN64_usb::usb_write(libusb_device_handle *handle, std::vector<uint8_t> output)
{
  int r;           //temp variable
  int byte_cnt;    //bytes counter

  //find out if kernel driver is attached
  if(libusb_kernel_driver_active(handle, 0) == 1)
  {
    ROS_INFO("EZN64: Kernel Driver Active");

    if(libusb_detach_kernel_driver(handle, 0) == 0) //detach it
      ROS_INFO("EZN64: Kernel Driver Detached!");
  }

  r = libusb_claim_interface(handle, 0); //claim interface 0 (the first) of device (mine had jsut 1)

  if(r < 0) ROS_INFO("EZN64: Cannot Claim Interface");

  //Write

  //Convert std::vector<uint8_t> to unsigned char array as requested by libusb API
  unsigned char *data_out = new unsigned char[output.size()];
  for(size_t i = 0; i < output.size(); i++)
  {
    data_out[i] = output[i];
    ROS_DEBUG("Output[%d]: %x", i,data_out[i]);
  }

  r = libusb_bulk_transfer(handle, 1, data_out, output.size(), &byte_cnt, LIBUSB_TIMEOUT);
  if (byte_cnt == 0) ROS_ERROR("EZN64: Write error");

  //Release interface

  r = libusb_release_interface(handle, 0);  //release the claimed interface
  if(r != 0)
    ROS_ERROR("EZN64: Cannot release the claimed interface");
   
  delete[] data_out;

}

std::vector<uint8_t>
EZN64_usb::usb_read(libusb_device_handle *handle)
{
  int r;           //temp variable
  int byte_cnt;    //bytes counter

  //find out if kernel driver is attached
  if(libusb_kernel_driver_active(handle, 0) == 1)
  {
    ROS_INFO("EZN64: Kernel Driver Active");

    if(libusb_detach_kernel_driver(handle, 0) == 0) //detach it
      ROS_INFO("EZN64: Kernel Driver Detached!");
  }

  r = libusb_claim_interface(handle, 0); //claim interface 0 (the first) of device (mine had just 1)

  if(r < 0) ROS_ERROR("EZN64: Cannot Claim Interface");

  //Read
  std::vector<uint8_t> input;
  unsigned char *data_in = new unsigned char[INPUT_BUFFER_SIZE];

  r = libusb_bulk_transfer(handle, LIBUSB_ENDPOINT, data_in, INPUT_BUFFER_SIZE, &byte_cnt, LIBUSB_TIMEOUT);
  if(r == 0)
  {
    for(size_t i = 0; i < byte_cnt; i++)
    {
      input.push_back(data_in[i]);
      ROS_DEBUG("Input[%d]: %x", i, data_in[i]);
    }
  }

  //Release interface
  r = libusb_release_interface(handle, 0);  //release the claimed interface
  if(r != 0)
    ROS_ERROR("EZN64: Cannot release the claimed interface");
   
  delete[] data_in;
  return input;
}


void
EZN64_usb::print_libusb_dev(libusb_device *dev)
{
  libusb_device_descriptor desc;
  int r = libusb_get_device_descriptor(dev, &desc);
  if (r < 0) 
  {
    ROS_ERROR("EZN64: failed to get device descriptor");
    return;
  }

  if((int)desc.bNumConfigurations > 0)
  {
    ROS_INFO_STREAM("EZN64: Number of possible configurations: " << (int)desc.bNumConfigurations);
    ROS_INFO_STREAM("EZN64: VendorID: "  << desc.idVendor);
    ROS_INFO_STREAM("EZN64: ProductID: " << desc.idProduct);

    libusb_config_descriptor *config;
    libusb_get_config_descriptor(dev, 0, &config);
    ROS_INFO_STREAM("EZN64: Interfaces: " << (int)config->bNumInterfaces);

    const libusb_interface *inter;
    const libusb_interface_descriptor *interdesc;
    const libusb_endpoint_descriptor *epdesc;

    for(int i = 0; i < (int)config->bNumInterfaces; i++) 
    {
      inter = &config->interface[i];
      ROS_DEBUG_STREAM("Number of alternate settings: " << inter->num_altsetting );
      for(int j = 0; j < inter->num_altsetting; j++)
      {
        interdesc = &inter->altsetting[j];
        ROS_DEBUG_STREAM("Interface Number: " << (int)interdesc->bInterfaceNumber);
        ROS_DEBUG_STREAM("Number of endpoints: " << (int)interdesc->bNumEndpoints);
        for(int k = 0; k < (int)interdesc->bNumEndpoints; k++) 
        {
          epdesc = &interdesc->endpoint[k];
          ROS_DEBUG_STREAM("Descriptor Type: "<< (int)epdesc->bDescriptorType);
          ROS_DEBUG_STREAM("EP Address: "<< (int)epdesc->bEndpointAddress);
        }
      }
    }
    libusb_free_config_descriptor(config);
  }
  else
  {
    ROS_ERROR("No Schunk EZN64 gripper found! Aborting...");
    exit(-1);
  }
    

}

float
EZN64_usb::IEEE_754_to_float(uint8_t *raw)
{
  int sign = (raw[0] >> 7) ? -1 : 1;
  int8_t exponent = (raw[0] << 1) + (raw[1] >> 7) - 126;

  uint32_t fraction_bits = ((raw[1] & 0x7F) << 16) + (raw[2] << 8) + raw[3];

  float fraction = 0.5f;
  for (uint8_t ii = 0; ii < 24; ++ii)
    fraction += ldexpf((fraction_bits >> (23 - ii)) & 1, -(ii + 1));

  float significand = sign * fraction;

  return ldexpf(significand, exponent);
}

void
EZN64_usb::float_to_IEEE_754(float position, unsigned int *output_array)
{
  unsigned char *p_byte = (unsigned char*)(&position);

  for(size_t i = 0; i < sizeof(float); i++)
    output_array[i] = (static_cast<unsigned int>(p_byte[i]));
}

} //schunk_ezn64

#endif //EZN64_USB_CONTROL_LIB_CPP
