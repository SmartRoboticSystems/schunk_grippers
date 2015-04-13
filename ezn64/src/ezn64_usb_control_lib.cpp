/*********************************************************************************************//**
* @file ezn64_usb_control_lib.cpp
*
* class ezn64_usb for USB communication between ROS and EZN64 gripper
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
#ifndef EZN64_USB_CONTROL_LIB_CPP
#define EZN64_USB_CONTROL_LIB_CPP

#include <ezn64_usb_control.h>

EZN64_usb::EZN64_usb(ros::NodeHandle *nh) :
    act_position(-1),
    ezn64_error(-1)
{
    //Read launch file params
    nh->getParam("ezn64/gripper_id", gripper_id);
    nh->getParam("ezn64/vendor_id", vendor_id);
    nh->getParam("ezn64/product_id", product_id);

    //Initialize USB and look for Schunk EZN64 controller
    ezn64_dev = find_ezn64_dev(vendor_id, product_id);
    print_libusb_dev(ezn64_dev);
    ezn64_handle = open_ezn64_dev(ezn64_dev);

    int r;

    //Get initial state and discard input buffer
    while (ezn64_error == -1)
       ezn64_error = get_state(ezn64_handle);
       ros::Duration(0.5).sleep();

    while(act_position == -1)
       act_position = get_position(ezn64_handle);
       ros::Duration(0.5).sleep();


}

EZN64_usb::~EZN64_usb()
{
      close_ezn64_dev(ezn64_handle, usb_context);
}

int
EZN64_usb::reference(libusb_device_handle *handle)
{
    std::cout << "EZN64 INFO: Referencing" << std::endl;
    std::vector<uint8_t> output;
    output.push_back(0x01);           //D-Len
    output.push_back(0x92);           //Command reference

    //Send message to the module
    std::vector<uint8_t> input;
    usb_write(handle, output);

}

float
EZN64_usb::set_position(libusb_device_handle *handle, int goal_position, float act_position)
{
    std::cout << "EZN64 IFNO: Moving from: " << act_position << " [mm] to " << goal_position << " [mm]" << std::endl;

    uint32_t ieee754_position_tbl[13] = {
    0x00000000, 0x3f800000, 0x40000000, 0x40400000, 0x40800000,
    0x40a00000, 0x40c00000, 0x40e00000, 0x41000000, 0x41100000,
    0x41200000, 0x41300000, 0x41400000};

    std::vector<uint8_t> output;
    output.push_back(0x05);                                         //D-Len
    output.push_back(0xb0);                                         //Command mov pos

    output.push_back( ieee754_position_tbl[goal_position] & 0x000000ff);         //Position first byte
    output.push_back((ieee754_position_tbl[goal_position] & 0x0000ff00) >> 8);   //Position second byte
    output.push_back((ieee754_position_tbl[goal_position] & 0x00ff0000) >> 16);  //Position third byte
    output.push_back((ieee754_position_tbl[goal_position] & 0xff000000) >> 24);  //Position fourth byte

    //Send message to the module and recieve response
    usb_write(handle, output);

    bool position_reached = false;
    std::vector<uint8_t> input;

    while(position_reached == false)
    {
        input = usb_read(handle);
        for(size_t i = 0; i < input.size(); i++)
            if((input[i] == 0x05) && (input[i+1] == 0x94))
            {
                position_reached = true;
                uint8_t raw[4] = {input[i+5], input[i+4], input[i+3], input[i+2]};
                act_position = IEEE_754_to_float(raw);
                std::cout << "EZN64 INFO: Reached position: " << act_position << std::endl;
                return act_position;
            }

        ros::Duration(0.1).sleep();
    }


}

uint8_t
EZN64_usb::get_state(libusb_device_handle *handle)
{
    std::cout << "EZN64 INFO: Reading current module state..." << std::endl;
    std::vector<uint8_t> output;
    output.push_back(0x06);                 //D-Len
    output.push_back(0x95);                 //Command get state
    output.push_back(0x00);
    output.push_back(0x00);
    output.push_back(0x00);
    output.push_back(0x00);
    output.push_back(0x01);

    //Send get_state message to the module and recieve response
    usb_write(handle, output);

    ros::Duration(0.1).sleep();
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

           int ezn64_error = input[get_state_index + 7];

           switch (ezn64_error)
           {
                   case 0x00: std::cout << "EZN64 INFO: No error detected" << std::endl; break;
                   case 0xC8: std::cout << "EZN65 INFO: Error: 0xC8 detected: Wrong ramp type" << std::endl; break;
                   case 0xD2: std::cout << "EZN65 INFO: Error: 0xD2 detected: Config memory" << std::endl; break;
                   case 0xD3: std::cout << "EZN65 INFO: Error: 0xD3 detected: Program memory" << std::endl; break;
                   case 0xD4: std::cout << "EZN65 INFO: Error: 0xD4 detected: Invalid phrase" << std::endl; break;
                   case 0xD5: std::cout << "EZN65 INFO: Error: 0xD5 detected: Soft low" << std::endl; break;
                   case 0xD6: std::cout << "EZN65 INFO: Error: 0xD6 detected: Soft high" << std::endl; break;
                   case 0xD7: std::cout << "EZN65 INFO: Error: 0xD7 detected: Pressure" << std::endl; break;
                   case 0xD8: std::cout << "EZN65 INFO: Error: 0xD8 detected: Service required" << std::endl; break;
                   case 0xD9: std::cout << "EZN65 INFO: Error: 0xD9 detected: Emergency stop" << std::endl; break;
                   case 0xDA: std::cout << "EZN65 INFO: Error: 0xDA detected: Tow" << std::endl; break;
                   case 0xE4: std::cout << "EZN65 INFO: Error: 0xE4 detected: Too fast" << std::endl; break;
                   case 0xEC: std::cout << "EZN65 INFO: Error: 0xEC detected: Math error" << std::endl; break;
                   case 0xDB: std::cout << "EZN65 INFO: Error: 0xDB detected: VPC3" << std::endl; break;
                   case 0xDC: std::cout << "EZN65 INFO: Error: 0xDC detected: Fragmentation" << std::endl; break;
                   case 0xDE: std::cout << "EZN65 INFO: Error: 0xDE detected: Current" << std::endl; break;
                   case 0xDF: std::cout << "EZN65 INFO: Error: 0xDF detected: I2T" << std::endl; break;
                   case 0xE0: std::cout << "EZN65 INFO: Error: 0xE0 detected: Initialize" << std::endl; break;
                   case 0xE1: std::cout << "EZN65 INFO: Error: 0xE1 detected: Internal" << std::endl; break;
                   case 0xE2: std::cout << "EZN65 INFO: Error: 0xE2 detected: Hard low" << std::endl; break;
                   case 0xE3: std::cout << "EZN65 INFO: Error: 0xE3 detected: Hard high" << std::endl; break;
                   case 0x70: std::cout << "EZN65 INFO: Error: 0x70 detected: Temp low" << std::endl; break;
                   case 0x71: std::cout << "EZN65 INFO: Error: 0x71 detected: Temp high" << std::endl; break;
                   case 0x72: std::cout << "EZN65 INFO: Error: 0x72 detected: Logic low" << std::endl; break;
                   case 0x73: std::cout << "EZN65 INFO: Error: 0x73 detected: Logic high" << std::endl; break;
                   case 0x74: std::cout << "EZN65 INFO: Error: 0x74 detected: Motor voltage low" << std::endl; break;
                   case 0x75: std::cout << "EZN65 INFO: Error: 0x75 detected: Motor voltage high" << std::endl; break;
                   case 0x76: std::cout << "EZN65 INFO: Error: 0x76 detected: Cable break" << std::endl; break;
                   case 0x78: std::cout << "EZN65 INFO: Error: 0x78 detected: Motor temp " << std::endl; break;
           }
           return((uint8_t)ezn64_error);
        }
        else
        {
            std::cout << "EZN64 WARN: Not get_state response" << std::endl;
            return(-1) ;
        }
     }
}

float
EZN64_usb::get_position(libusb_device_handle *handle)
{
    std::cout << "EZN64 INFO: Reading current module position..." << std::endl;
    std::vector<uint8_t> output;
    output.push_back(0x06);                 //D-Len
    output.push_back(0x95);                 //Command get state
    output.push_back(0x00);
    output.push_back(0x00);
    output.push_back(0x00);
    output.push_back(0x00);
    output.push_back(0x01);

    //Send get_state message to the module and recieve response
    usb_write(handle, output);

    ros::Duration(0.1).sleep();
    std::vector<uint8_t> input;
    input = usb_read(handle);

    //Detect reached position response
    float act_position;

    if(input.size() > 0)
    {
        for(size_t i = 0; i < input.size(); i++)
            if((input[i] == 0x07) && (input[i+1] == 0x95))
            {
                uint8_t raw[4] = {input[i+5], input[i+4], input[i+3], input[i+2]};
                act_position = IEEE_754_to_float(raw);
                std::cout << "EZN64 INFO: Actual position: " << act_position << std::endl;
                return(act_position);
            }
            else if ((input[i] == 0x05) && (input[i+1] == 0x94))
            {
                uint8_t raw[4] = {input[i+5], input[i+4], input[i+3], input[i+2]};
                act_position = IEEE_754_to_float(raw);
                std::cout << "EZN64 INFO: Actual position: " << act_position << std::endl;
                return(act_position);
            }
            else return(-1);
    }
    else
        {
            std::cout << "EZN64 WARN: Not get_state response" << std::endl;
            return(-1) ;
        }
}

int
EZN64_usb::stop(libusb_device_handle *handle)
{
    std::vector<uint8_t> output;
    output.push_back(0x01);                                         //D-Len
    output.push_back(0x91);                                         //Command stop

    //Send message to the module
    usb_write(handle, output);

    return 1;

}

int
EZN64_usb::acknowledge_error(libusb_device_handle *handle)
{
    std::vector<uint8_t> output;
    output.push_back(0x01);                                         //D-Len
    output.push_back(0x8b);                                         //Command cmd ack

    //Send message to the module
    usb_write(handle, output);

    ros::Duration(0.1).sleep();
    std::vector<uint8_t> input;
    input = usb_read(handle);


    //Read and process response
    if(input.size() > 0)
    {
        if(input[1] == 0x92)
        {
            if ((input[2] == 0x4f) && (input[3] == 0x4f)) return(0);
            else return(-1);
        }
        else std::cout << "EZN64 WARN: Not cmd_ack response" << std::endl;
    }
    else
    {
        std::cout << "EZN64 WARN: No response recieved" << std::endl;
        return(-1);
    }
}



///////////////////////////////////////////////////////////////
//CALLBACKS
///////////////////////////////////////////////////////////////

bool
EZN64_usb::reference_callback(ezn64::reference::Request &req,
                                 ezn64::reference::Response &res)
{
    std::cout << "EZN64 INFO: Reference Cmd recieved " << std::endl;
    if(req.reference_request== true)
        res.reference_response = reference(ezn64_handle);
    else
        res.reference_response = false;
}


bool
EZN64_usb::set_position_callback(ezn64::set_position::Request &req,
                                    ezn64::set_position::Response &res)
{
    std::cout << "EZN64 INFO: Set position Cmd recieved" << std::endl;

    //Check if goal request respects gripper limits <0-10> mm
    if ((req.goal_position >= 0) && (req.goal_position < 13))
    {
        std::cout << "EZN64 INFO: Goal accepted " << std::endl;
        act_position = set_position(ezn64_handle,req.goal_position, act_position);
        res.goal_accepted = true;
     }
     else
     {
         std::cout << "EZN64 WARN: Goal position rejected!" << std::endl;
         res.goal_accepted = false;
     }
}

bool
EZN64_usb::get_state_callback(ezn64::get_state::Request &req,
                                 ezn64::get_state::Response &res)
{
    std::cout << "EZN64 INFO: Get state request recieved" << std::endl;
    if (req.get_state_request == true)
        res.error_code = get_state(ezn64_handle);

}

bool
EZN64_usb::get_position_callback(ezn64::get_position::Request &req,
                                 ezn64::get_position::Response &res)
{
    std::cout << "EZN64 INFO: Get position request recieved" << std::endl;
    if (req.get_position_request == true)
        res.actual_position = get_position(ezn64_handle);

}

bool
EZN64_usb::acknowledge_error_callback(ezn64::acknowledge_error::Request &req,
                                         ezn64::acknowledge_error::Response &res)
{
    std::cout << "EZN64 INFO: Cmd Acknowledge error recieved" << std::endl;
    if(req.acknowledge_request == true)
       res.acknowledge_response = acknowledge_error(ezn64_handle);
    else
        res.acknowledge_response = false;
}


bool
EZN64_usb::stop_callback(ezn64::stop::Request &req,
                            ezn64::stop::Response &res)
{
    std::cout << "EZN64 INFO: Cmd Stop recieved" << std::endl;
    if(req.stop_request == true)
        res.stop_result = stop(ezn64_handle);
    else
        res.stop_result = false;
}

/////////////////////////////////////////////////////////
//LIBUSB FUNCTIONS
/////////////////////////////////////////////////////////

libusb_device*
EZN64_usb::find_ezn64_dev(int VendorID, int ProductID)
{
    int r;

    //Initialize USB session
    r= libusb_init(&usb_context);
    if (r < 0)
    {
        std::cout << "EZN64 ERROR: Init Error" << std::endl;
        return 0;
    }
    libusb_set_debug(usb_context,3);        //set verbosity level to 3, as suggested in the documentation

    //Get list of devices
    static size_t dev_cnt;          //number of devices
    libusb_device **devs;           //pointer to pointer of device used to retrieve a list of devices
    dev_cnt = libusb_get_device_list(usb_context, &devs);
    if(dev_cnt < 0)
    {
        std::cout << "EZN64 ERROR: Get device Error" << std::endl;
        return 0;
    }
    else
        std::cout << "EZN64 INFO: Number of devices: " << dev_cnt << std::endl;

    for(size_t i =0; i < dev_cnt; i++)
    {
        struct libusb_device_descriptor descriptor;
        if(libusb_get_device_descriptor(devs[i], &descriptor))
        {
            std::cout << "EZN64 ERROR: Not able to read descriptor" << std::endl;
            return 0;
        }
        if((descriptor.idVendor == VendorID) && (descriptor.idProduct == ProductID))
        {
            std::cout << "EZN64 INFO: Gripper found" << std::endl;
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
        std::cout << "EZN64 ERROR: Port not oppened! " <<std::endl;
        return 0;
    }
    else
    {
        std::cout << "EZN64 INFO: USB interface oppened successfully! " << std::endl;
        return handle;
    }

}

int
EZN64_usb::close_ezn64_dev(libusb_device_handle *handle, libusb_context *usb_context)
{
   std::cout << "EZN64 INFO: Closing USB port " << std::endl;
   libusb_close(handle);
   libusb_exit(usb_context);
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
        std::cout << "EZN64 INFO: Kernel Driver Active" << std::endl;

         if(libusb_detach_kernel_driver(handle, 0) == 0) //detach it
            std::cout << "EZN64 INFO: Kernel Driver Detached!" << std::endl;
    }

   r = libusb_claim_interface(handle, 0); //claim interface 0 (the first) of device (mine had jsut 1)

   if(r < 0) std::cout << "EZN64 INFO: Cannot Claim Interface" << std::endl;

   //Write

   //Convert std::vector<uint8_t> to unsigned char array as requested by libusb API
   unsigned char *data_out = new unsigned char[output.size()];
   for(size_t i = 0; i < output.size(); i++)
       data_out[i] = output[i];

   r = libusb_bulk_transfer(handle, 1, data_out, output.size(), &byte_cnt,1000);
   if (byte_cnt == 0) std::cout << "EZN64 ERROR: Write error" << std::endl;

   //Release interface

   r = libusb_release_interface(handle, 0);  //release the claimed interface
   if(r != 0)
   {
       std::cout << "EZN64 ERROR: Cannot release the claimed interface" << std::endl;
   }

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
        std::cout << "EZN64 INFO: Kernel Driver Active" << std::endl;

         if(libusb_detach_kernel_driver(handle, 0) == 0) //detach it
            std::cout << "EZN64 INFO: Kernel Driver Detached!" << std::endl;
    }

   r = libusb_claim_interface(handle, 0); //claim interface 0 (the first) of device (mine had jsut 1)

   if(r < 0) std::cout << "EZN64 INFO: Cannot Claim Interface" << std::endl;

   //Read
   std::vector<uint8_t> input;
   unsigned char *data_in = new unsigned char[512];
   ros::Duration(1).sleep();

   r = libusb_bulk_transfer(handle, 129, data_in, 512, &byte_cnt,1000);
   if(r == 0)
   {
        for(size_t i = 0; i < byte_cnt; i++)
        {
            input.push_back(data_in[i]);
            //ROS_INFO("Byte: %x", data_in[i]);
        }
   }
   else
   {
       std::cout << "EZN64 ERROR: Read response message error " << std::endl;
   }

   //Release interface

   r = libusb_release_interface(handle, 0);  //release the claimed interface
   if(r != 0)
   {
       std::cout << "EZN64 ERROR: Cannot release the claimed interface" << std::endl;
   }

   delete[] data_in;
   return input;
}


void
EZN64_usb::print_libusb_dev(libusb_device *dev)
{
   libusb_device_descriptor desc;
   int r = libusb_get_device_descriptor(dev, &desc);
   if (r < 0) {
       std::cout<<"EZN64 ERROR: failed to get device descriptor"<<std::endl;
       return;
    }

   std::cout << "EZN64 INFO: Number of possible configurations: " << (int)desc.bNumConfigurations<<"  " << std::endl;
   std::cout << "EZN64 INFO: VendorID: "  << desc.idVendor <<"  ";
   std::cout << " ProductID: " << desc.idProduct<< std::endl;

   libusb_config_descriptor *config;
   libusb_get_config_descriptor(dev, 0, &config);
   std::cout<<"EZN64 INFO: Interfaces: "<<(int)config->bNumInterfaces<< std::endl;

   const libusb_interface *inter;
   const libusb_interface_descriptor *interdesc;
   const libusb_endpoint_descriptor *epdesc;

#ifdef DEBUG

   for(int i=0; i<(int)config->bNumInterfaces; i++) {
       inter = &config->interface[i];
       std::cout<<"Number of alternate settings: "<<inter->num_altsetting<< std::endl;
       for(int j=0; j<inter->num_altsetting; j++) {
           interdesc = &inter->altsetting[j];
           std::cout << "Interface Number: " <<(int)interdesc->bInterfaceNumber << std::endl;
           std::cout << "Number of endpoints: " <<(int)interdesc->bNumEndpoints << std::endl;
           for(int k=0; k<(int)interdesc->bNumEndpoints; k++) {
               epdesc = &interdesc->endpoint[k];
               std::cout <<"Descriptor Type: "<<(int)epdesc->bDescriptorType << std::endl;
               std::cout <<"EP Address: "<<(int)epdesc->bEndpointAddress << std::endl;
               }
          }
  }
#endif

  libusb_free_config_descriptor(config);
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


#endif //EZN64_USB_CONTROL_LIB_CPP
