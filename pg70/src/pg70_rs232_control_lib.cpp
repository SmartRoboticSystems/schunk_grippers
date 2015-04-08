/*********************************************************************************************//**
* @file pg70_rs232_control_lib.cpp
*
* class pg70_serial for serial communication between ROS and PG70 gripper
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
#ifndef PG70_RS232_CONTROL_LIB_CPP
#define PG70_RS232_CONTROL_LIB_CPP

#include <pg70_rs232_control.h>

PG70_serial::PG70_serial(ros::NodeHandle *nh)
{
    //Read launch file params
    nh->getParam("pg70/portname", portname);
    nh->getParam("pg70/baudrate", baudrate);
    nh->getParam("pg70/gripper_id", gripper_id);

    //Initialize and open serial port
    com_port = new serial::Serial (portname, (uint32_t)baudrate, serial::Timeout::simpleTimeout(100));

    if (com_port->isOpen()) std::cout << "PG70 INFO: Serial port " << portname << " openned successfully" << std::endl;
    else                    std::cout << "PG70 INFO: Serial port " << portname<< " not opened" << std::endl;


    //IEEE754 table to convert uint32_t values storing gripper position to <0-69> mm
    uint32_t position_data[70] = {
    0x00000000, 0x3f800000, 0x40000000, 0x40400000, 0x40800000,
    0x40a00000, 0x40c00000, 0x40e00000, 0x41000000, 0x41100000,
    0x41200000, 0x41300000, 0x41400000, 0x41500000, 0x41600000,
    0x41700000, 0x41800000, 0x41880000, 0x41900000, 0x41980000,
    0x41a00000, 0x41a80000, 0x41b00000, 0x41b80000, 0x41c00000,
    0x41c80000, 0x41d00000, 0x41d80000, 0x41e00000, 0x41e80000,
    0x41f00000, 0x41f80000, 0x42000000, 0x42040000, 0x42080000,
    0x420c0000, 0x42100000, 0x42140000, 0x42180000, 0x421c0000,
    0x42200000, 0x42240000, 0x42280000, 0x422c0000, 0x42300000,
    0x42340000, 0x42380000, 0x423c0000, 0x42400000, 0x42440000,
    0x42480000, 0x424c0000, 0x42500000, 0x42540000, 0x42580000,
    0x425c0000, 0x42600000, 0x42640000, 0x42680000, 0x426c0000,
    0x42700000, 0x42740000, 0x42780000, 0x427c0000, 0x42800000,
    0x42820000, 0x42840000, 0x42860000, 0x42880000, 0x428a0000,
    };
    ieee754_position_tbl = position_data;

    //IEEE754 table to convert uint32_t values storing gripper position to <0 -82> mm/s
    uint32_t velocity_data[83] = {
    0x00000000, 0x3f800000, 0x40000000, 0x40400000, 0x40800000,
    0x40a00000, 0x40c00000, 0x40e00000, 0x41000000, 0x41100000,
    0x41200000, 0x41300000, 0x41400000, 0x41500000, 0x41600000,
    0x41700000, 0x41800000, 0x41880000, 0x41900000, 0x41980000,
    0x41a00000, 0x41a80000, 0x41b00000, 0x41b80000, 0x41c00000,
    0x41c80000, 0x41d00000, 0x41d80000, 0x41e00000, 0x41e80000,
    0x41f00000, 0x41f80000, 0x42000000, 0x42040000, 0x42080000,
    0x420c0000, 0x42100000, 0x42140000, 0x42180000, 0x421c0000,
    0x42200000, 0x42240000, 0x42280000, 0x422c0000, 0x42300000,
    0x42340000, 0x42380000, 0x423c0000, 0x42400000, 0x42440000,
    0x42480000, 0x424c0000, 0x42500000, 0x42540000, 0x42580000,
    0x425c0000, 0x42600000, 0x42640000, 0x42680000, 0x426c0000,
    0x42700000, 0x42740000, 0x42780000, 0x427c0000, 0x42800000,
    0x42820000, 0x42840000, 0x42860000, 0x42880000, 0x428a0000,
    0x428c0000, 0x428e0000, 0x42900000, 0x42920000, 0x42940000,
    0x42960000, 0x42980000, 0x429a0000, 0x429c0000, 0x429e0000,
    0x42a00000, 0x42a20000, 0x42a40000 };
    ieee754_velocity_tbl = velocity_data;

     //IEEE754 table to convert uint32_t values storing gripper position to <0..10..20..320> mm/s2
    uint32_t acceleration_data[33] = {
    0x00000000, 0x41200000, 0x41a00000, 0x41f00000, 0x42200000,
    0x42480000, 0x42700000, 0x428c0000, 0x42a00000, 0x42b40000,
    0x42c80000, 0x42dc0000, 0x42f00000, 0x43020000, 0x430c0000,
    0x43160000, 0x43200000, 0x432a0000, 0x43340000, 0x433e0000,
    0x43480000, 0x43520000, 0x435c0000, 0x43660000, 0x43700000,
    0x437a0000, 0x43820000, 0x43870000, 0x438c0000, 0x43910000,
    0x43960000, 0x439b0000, 0x43a00000};
    ieee754_acceleration_tbl = acceleration_data;


    set_position(com_port, 20,60,100);
    ros::Duration(5).sleep();

    int temp_position;
    temp_position = get_state(com_port);
    std::cout << "Current position: " << temp_position << std::endl;

}

PG70_serial::~PG70_serial()
{
    com_port->close();      //Close port
    delete com_port;        //delete object
}

bool
PG70_serial::reference(serial::Serial *port)
{
    std::vector<uint8_t> output;
    output.push_back(0x05);           //message from master to module
    output.push_back(gripper_id);     //module id
    output.push_back(0x01);           //D-Len
    output.push_back(0x92);           //Command reference

    //Checksum calculation
    uint16_t crc = 0;

    for(size_t i = 0; i < output.size(); i++)
       crc = CRC16(crc,output[i]);

    //Add checksum to the output buffer
    output.push_back(crc & 0x00ff);
    output.push_back((crc & 0xff00) >> 8);

    //Send message to the module
    port->flushInput();
    port->write(output);

    //Read and process response
     std::vector<uint8_t> input;
     ros::Duration(0.2).sleep();
     port->read(input,8);

     if(input.size() > 0){
         std::cout << "PG70 INFO: Cmd Reference response recieved " << std::endl;
         for(size_t i = 0; i < input.size(); i++)
             ROS_INFO("Byte %i: %x", i, input[i]);
         if(input[0] == 0x07) {
             if(input[1] == gripper_id) {
                 if(input[3] == 0x92)
                 {
                     if((input[4] == 0x4f) && (input[5] == 0x4f))
                     {
                         std::cout << "Referencing..." << std::cout;
                         return(true);
                     }
                 }
                 else
                 {
                     std::cout << "PG70 WARN: Not get_state response" << std::endl;
                     return(false);
                 }
             }
             else
             {
                 std::cout << "PG70 WARN: Wrong module id number" << std::endl;
                 return(false);
             }
         }
         else
         {
             std::cout << "PG70 WARN: Not 'slave -> master' package" << std::endl;
             return(false);
         }
     }
     else
     {
         std::cout <<"PG70 WARN: No response recieved" << std::endl;
         return(false);
     }



}

void
PG70_serial::set_position(serial::Serial *port, int position, int velocity, int acceleration)
{

    std::vector<uint8_t> output;
    output.push_back(0x05);                                         //message from master to module
    output.push_back(gripper_id);                                   //module id
    output.push_back(0x0D);                                         //D-Len
    output.push_back(0xb0);                                         //Command mov pos
    //Position <0-69>mm
    output.push_back( ieee754_position_tbl[position] & 0x000000ff);         //Position first byte
    output.push_back((ieee754_position_tbl[position] & 0x0000ff00) >> 8);   //Position second byte
    output.push_back((ieee754_position_tbl[position] & 0x00ff0000) >> 16);  //Position third byte
    output.push_back((ieee754_position_tbl[position] & 0xff000000) >> 24);  //Position fourth byte

    //Velocity<0-82>mm/s
    output.push_back( ieee754_velocity_tbl[velocity] & 0x000000ff);         //Velocity first byte
    output.push_back((ieee754_velocity_tbl[velocity] & 0x0000ff00) >> 8);   //Velocity second byte
    output.push_back((ieee754_velocity_tbl[velocity] & 0x00ff0000) >> 16);  //Velocity third byte
    output.push_back((ieee754_velocity_tbl[velocity] & 0xff000000) >> 24);  //Velocity fourth byte

    //Acceleration<0-320>mm/s2
    output.push_back( ieee754_acceleration_tbl[acceleration/10] & 0x000000ff);         //Acceleration first byte
    output.push_back((ieee754_acceleration_tbl[acceleration/10] & 0x0000ff00) >> 8);   //Acceleration second byte
    output.push_back((ieee754_acceleration_tbl[acceleration/10] & 0x00ff0000) >> 16);  //Acceleration third byte
    output.push_back((ieee754_acceleration_tbl[acceleration/10] & 0xff000000) >> 24);  //Acceleration fourth byte

    //Checksum calculation
    uint16_t crc = 0;

    for(size_t i = 0; i < output.size(); i++)
       crc = CRC16(crc,output[i]);

    //Add checksum to the output buffer
    output.push_back(crc & 0x00ff);
    output.push_back((crc & 0xff00) >> 8);

    //Send message to the module
    port->write(output);

    //Console output (DEBUG purposes)
    std::cout << "Output: " << std::endl;
    for(size_t i=0; i < output.size(); i++)
        ROS_INFO("Byte %i: %x", i, output[i]);
\
}

int
PG70_serial::get_state(serial::Serial *port)
{
    std::vector<uint8_t> output;
    output.push_back(0x05);                 //message from master to module
    output.push_back(gripper_id);           //module id
    output.push_back(0x01);                 //D-Len
    output.push_back(0x95);                 //Command get state

    //Checksum calculation
    uint16_t crc = 0;

    for(size_t i = 0; i < output.size(); i++)
       crc = CRC16(crc,output[i]);

    //Add checksum to the output buffer
    output.push_back(crc & 0x00ff);
    output.push_back((crc & 0xff00) >> 8);

    //Clear input buffer and send get_state message to the module
    port->flushInput();
    port->write(output);

   //Read and process current state message
    std::vector<uint8_t> input;
    ros::Duration(0.2).sleep();
    port->read(input,20);

    if(input.size() > 0){
        std::cout << "PG70 INFO: Response Recieved " << std::endl;
        for(size_t i = 0; i < input.size(); i++)
            ROS_INFO("Byte %i: %x", i, input[i]);
        if(input[0] == 0x07) {
            if(input[1] == gripper_id) {
                if(input[3] == 0x95)
                {
                    uint32_t position;
                    position = (input[7] << 24) | ((input[6] & 0xf0) << 16);         //Merge 2 most significant position bytes to single variable
                    for(size_t i = 0; i <= 70; i++)                                  //Lookup real position in IEEE754 table
                      if(position == ieee754_position_tbl[i]) return(i+1);           //return index corresponding to position in mm
                }
                else std::cout << "PG70 WARN: Not get_state response" << std::endl;
            }
            else std::cout << "PG70 WARN: Wrong module id number" << std::endl;
        }
        else std::cout << "PG70 WARN: Not 'slave -> master' package" << std::endl;
    }
    else std::cout <<"PG70 WARN: No response recieved" << std::endl;
}

bool
PG70_serial::reference_callback(pg70::reference::Request &req,
                                pg70::reference::Response &res)
{
    std::cout << "PG70 INFO: Reference Cmd recieved " << std::endl;
    if(req.reference_request== true)
        res.reference_response = reference(com_port);
    else
        res.reference_response = false;
}


bool
PG70_serial::set_position_callback(pg70::set_position::Request &req,
                                   pg70::set_position::Response &res)
{
    std::cout << "PG70 INFO: Set position Cmd recieved" << std::endl;

    //Check if goal request respects gripper limits <0-70> mm
    if ((req.goal_position > 0) && (req.goal_position < 70))
    {
        if((req.goal_velocity > 0) && (req.goal_velocity < 83))
        {
            if((req.goal_acceleration > 0) && (req.goal_acceleration <= 320))
            {
                std::cout << "PG70 INFO: Goal accepted " << std::endl;
                set_position(com_port,req.goal_position, req.goal_velocity, req.goal_acceleration);
                res.goal_accepted = true;
            }
            else
            {
                std::cout << "PG70 WARN: Goal acceleration rejected!" << std::endl;
                res.goal_accepted = false;
            }
        }
        else
        {
           std::cout << "PG70 WARN: Goal velocity rejected!" << std::endl;
           res.goal_accepted = false;
        }
    }
    else
    {
         std::cout << "PG70 WARN: Goal position rejected!" << std::endl;
         res.goal_accepted = false;
    }
}
bool
PG70_serial::get_state_callback(pg70::get_state::Request &req,
                                pg70::get_state::Response &res)
{
    std::cout << "PG70 INFO: Get state request recieved" << std::endl;
    if (req.get_state_request == true)
        res.actual_position = get_state(com_port);

}

bool
PG70_serial::acknowledge_error_callback(pg70::acknowledge_error::Request &req,
                                        pg70::acknowledge_error::Response &res)
{
    std::cout << "PG70 INFO: Cmd Acknowledge error recieved" << std::endl;
    if(req.acknowledge_request == true)
       res.acknowledge_response = acknowledge_error(com_port);
    else
        res.acknowledge_response = false;

}

bool
PG70_serial::stop_callback(pg70::stop::Request &req,
                           pg70::stop::Response &res)
{
    std::cout << "PG70 INFO: Cmd Stop recieved" << std::endl;
    if(req.stop_request == true)
        res.stop_result = stop(com_port);
    else
        res.stop_result = false;
}

bool
PG70_serial::stop(serial::Serial *port)
{
    std::vector<uint8_t> output;
    output.push_back(0x05);                                         //message from master to module
    output.push_back(gripper_id);                                   //module id
    output.push_back(0x01);                                         //D-Len
    output.push_back(0x91);                                         //Command stop

    //Checksum calculation
    uint16_t crc = 0;

    for(size_t i = 0; i < output.size(); i++)
       crc = CRC16(crc,output[i]);

    //Add checksum to the output buffer
    output.push_back(crc & 0x00ff);
    output.push_back((crc & 0xff00) >> 8);

    //Clear input buffer and send message to the module
    port->flushInput();
    port->write(output);

    //Read and process response
    std::vector<uint8_t> input;
    port->read(input,7);

    if(input[0] == 0x07) {
        if(input[1] == gripper_id) {
            if(input[3] == 0x91) {
                if((input[4] == 0x4B) && (input[5] == 0x4b)){
                    std::cout << "PG70 INFO: Stop movement successfull " << std::endl;
                    return(true);
                }
                else{
                    std::cout << "PG70 WARN: Stop movement not successfull " << std::endl;
                    return(false);
                }

            }
            else std::cout << "PG70 WARN: Not stop cmd response" << std::endl;
        }
        else std::cout << "PG70 WARN: Wrong module id number in response" << std::endl;
    }
    else std::cout << "PG70 WARN: Not slave -> master package" << std::endl;
}

bool
PG70_serial::acknowledge_error(serial::Serial *port)
{
    std::vector<uint8_t> output;
    output.push_back(0x05);                                         //message from master to module
    output.push_back(gripper_id);                                   //module id
    output.push_back(0x01);                                         //D-Len
    output.push_back(0x8b);                                         //Command cmd ack

    //Checksum calculation
    uint16_t crc = 0;

    for(size_t i = 0; i < output.size(); i++)
       crc = CRC16(crc,output[i]);

    //Add checksum to the output buffer
    output.push_back(crc & 0x00ff);
    output.push_back((crc & 0xff00) >> 8);

    //Clear input buffer and send message to the module
    port->flushInput();
    port->write(output);

    //Read and process response
    std::vector<uint8_t> input;
    port->read(input,8);

    if(input[0] == 0x07) {
        if(input[1] == gripper_id) {
            if(input[3] == 0x8b)
            {
                if ((input[4] == 0x4f) && (input[5] == 0x4f)) return(true);
                else return(false);
            }
            else std::cout << "PG70 WARN: Not cmd_ack response" << std::endl;
        }
        else std::cout << "PG70 WARN: Wrong module id number in response" << std::endl;
    }
    else std::cout << "PG70 WARN: Not slave -> master package" << std::endl;

}


//CRC Table according to Schunk MotionControl.pdf
uint16_t
PG70_serial::CRC16(uint16_t crc, uint16_t data)
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

#endif //PG70_RS232_CONTROL_LIB_CPP
