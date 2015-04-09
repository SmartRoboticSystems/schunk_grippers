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

EZN64_usb::EZN64_usb(ros::NodeHandle *nh)
{
    //Read launch file params
    nh->getParam("ezn64/gripper_id", gripper_id);
    nh->getParam("ezn64/vendor_id", vendor_id);
    nh->getParam("ezn64/product_id", product_id);

    //Initialize USB and look for Schunk EZN64 controller
    int r;
    ezn64_dev = find_ezn64_dev(vendor_id, product_id);
    print_libusb_dev(ezn64_dev);
    ezn64_handle = open_ezn64_dev(ezn64_dev);

    r = acknowledge_error(ezn64_handle);
    ros::Duration(2).sleep();
    r = set_position(ezn64_handle, 12);
    ros::Duration(5).sleep();
    r = reference(ezn64_handle);
    ros::Duration(2).sleep();

    r = close_ezn64_dev(ezn64_handle, usb_context);

}

EZN64_usb::~EZN64_usb()
{

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
    input = usb_transaction(handle, output, 4);

     if(input.size() > 0)
     {
        if(input[1] == 0x92)
         {
            if((input[2] == 0x4f) && (input[3] == 0x4b))
            {
                std::cout << "Referencing successful " << std::cout;
                return 0;
            }
         }
         else
         {
                std::cout << "EZN64 WARN: Not get_state response" << std::endl;
                return -1;
         }
     }
     else
     {
         std::cout <<"EZN64 WARN: No response recieved" << std::endl;
         return -1;
     }
}

int
EZN64_usb::set_position(libusb_device_handle *handle, int position)
{

    uint32_t ieee754_position_tbl[13] = {
    0x00000000, 0x3f800000, 0x40000000, 0x40400000, 0x40800000,
    0x40a00000, 0x40c00000, 0x40e00000, 0x41000000, 0x41100000,
    0x41200000, 0x41300000, 0x41400000};

    std::vector<uint8_t> output;
    output.push_back(0x05);                                         //D-Len
    output.push_back(0xb0);                                         //Command mov pos

    output.push_back( ieee754_position_tbl[position] & 0x000000ff);         //Position first byte
    output.push_back((ieee754_position_tbl[position] & 0x0000ff00) >> 8);   //Position second byte
    output.push_back((ieee754_position_tbl[position] & 0x00ff0000) >> 16);  //Position third byte
    output.push_back((ieee754_position_tbl[position] & 0xff000000) >> 24);  //Position fourth byte

    //Send message to the module and recieve response
    std::vector<uint8_t> input;
    input = usb_transaction(handle, output,6);
    if(input.size() > 0)   return 0;
    else    return -1;
}

int
EZN64_usb::get_state(libusb_device_handle *handle)
{
    uint32_t ieee754_position_tbl[13] = {
    0x00000000, 0x3f800000, 0x40000000, 0x40400000, 0x40800000,
    0x40a00000, 0x40c00000, 0x40e00000, 0x41000000, 0x41100000,
    0x41200000, 0x41300000, 0x41400000};

    std::vector<uint8_t> output;
    output.push_back(0x01);                 //D-Len
    output.push_back(0x95);                 //Command get state

    //Send get_state message to the module and recieve response
    std::vector<uint8_t> input;
    input = usb_transaction(handle, output, 15);

   //Read and process current state message
    if(input.size() > 0)
    {
        if(input[1] == 0x95)
        {
           uint32_t position;
           position = (input[5] << 24) | ((input[4] & 0xf0) << 16);        //Merge 2 most significant position bytes to single variable
           for(size_t i = 0; i <= 13; i++)                                  //Lookup real position in IEEE754 table
                if(position == ieee754_position_tbl[i]) return(i+1);          //return index corresponding to position in mm
        }
        else
        {
            std::cout << "EZN64 WARN: Not get_state response" << std::endl;
            return -1;
        }
     }
    else
    {
        std::cout << "EZN64 WARN: Wrong module id number" << std::endl;
        return -1;
    }

}

int
EZN64_usb::stop(libusb_device_handle *handle)
{
    std::vector<uint8_t> output;
    output.push_back(0x01);                                         //D-Len
    output.push_back(0x91);                                         //Command stop

    //Send message to the module
    std::vector<uint8_t> input;
    input = usb_transaction(handle, output, 4);

    //Read and process response
    if(input.size() > 0)
    {
        if(input[1] == 0x91)
        {
            if((input[2] == 0x4f) && (input[3] == 0x4b))
            {
                std::cout << "EZN64 INFO: Stop movement successfull " << std::endl;
                return 0;
            }
            else
            {
                std::cout << "EZN64 WARN: Stop movement not successfull " << std::endl;
                return -1;
            }
        }
        else
        {
             std::cout << "EZN64 WARN: Not stop cmd response" << std::endl;
             return -1;
        }
    }
    else
    {
        std::cout << "EZN64 WARN: No response recieved" << std::endl;
        return -1;
    }
}

int
EZN64_usb::acknowledge_error(libusb_device_handle *handle)
{
    std::vector<uint8_t> output;
    output.push_back(0x01);                                         //D-Len
    output.push_back(0x8b);                                         //Command cmd ack

    //Send message to the module
    std::vector<uint8_t> input;
    input = usb_transaction(handle, output, 4);

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
    if ((req.goal_position > 0) && (req.goal_position < 13))
    {
        std::cout << "EZN64 INFO: Goal accepted " << std::endl;
        set_position(ezn64_handle,req.goal_position);
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
        res.actual_position = get_state(ezn64_handle);

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

std::vector<uint8_t>
EZN64_usb::usb_transaction(libusb_device_handle *handle, std::vector<uint8_t> output, int num_bytes_to_read)
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

   std::cout << "EZN64 INFO: Claimed Interface"<< std::endl;

   //Write

   //Convert std::vector<uint8_t> to unsigned char array as requested by libusb API
   unsigned char *data_out = new unsigned char[output.size()];
   for(size_t i = 0; i < output.size(); i++)
       data_out[i] = output[i];

   r = libusb_bulk_transfer(handle, 1, data_out, output.size(), &byte_cnt,1000);
   if ((r == 0) && (byte_cnt == output.size()))
       std::cout << "EZN64 INFO: Sent " << byte_cnt << " bytes" << std::endl;
   else
       std::cout << "EZN64 ERROR: Write error" << std::endl;

   //Read

   std::vector<uint8_t> input;
   unsigned char *data_in = new unsigned char[512];

   r = libusb_bulk_transfer(handle, 129, data_in, 512, &byte_cnt,1000);
   std::cout << "R: " << r << std::endl;
   if(r == 0)
   {
        std::cout << "EZN64 INFO: " << byte_cnt << " bytes recieved" << std::endl;
        for(size_t i = 0; i < num_bytes_to_read; i++)
        {
            input.push_back(data_in[i]);
            ROS_INFO("Byte: %x", data_in[i]);
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

   std::cout << "EZN64 INFO: Released Interface " << std::endl;
   delete[] data_out;
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
   std::cout << "VendorID: "  << desc.idVendor <<"  ";
   std::cout << "ProductID: " << desc.idProduct<< std::endl;

   libusb_config_descriptor *config;
   libusb_get_config_descriptor(dev, 0, &config);
   std::cout<<"EZN64 INFO: Interfaces: "<<(int)config->bNumInterfaces<< std::endl;

   const libusb_interface *inter;
   const libusb_interface_descriptor *interdesc;
   const libusb_endpoint_descriptor *epdesc;

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
  std::cout << std::endl << std::endl << std::endl;
  libusb_free_config_descriptor(config);
}


//CRC Table according to Schunk MotionControl.pdf
uint16_t
EZN64_usb::CRC16(uint16_t crc, uint16_t data)
{
    const uint16_t tbl[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    return(((crc & 0xFF00) >> 8)  ^ tbl[(crc & 0x00FF) ^ (data & 0x00FF)]);

}

#endif //EZN64_USB_CONTROL_LIB_CPP
