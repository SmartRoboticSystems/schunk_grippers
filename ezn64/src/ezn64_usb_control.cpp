/*********************************************************************************************//**
* @file EZN64_usb_control.cpp
*
* main ezn64_usb control source
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

#include <ezn64_usb_control.h>
int 
main(int argc, char *argv[])
{
    ros::init(argc, argv, "ezn64_usb_control");
    ros::NodeHandle nh;

    EZN64_usb gripper(&nh);

    ros::ServiceServer reference_service         = nh.advertiseService("reference", &EZN64_usb::reference_callback, &gripper);
    ros::ServiceServer set_position_service      = nh.advertiseService("set_position", &EZN64_usb::set_position_callback, &gripper);
    ros::ServiceServer get_state_service         = nh.advertiseService("get_state", &EZN64_usb::get_state_callback, &gripper);
    ros::ServiceServer get_position_service      = nh.advertiseService("get_position", &EZN64_usb::get_position_callback, &gripper);
    ros::ServiceServer acknowledge_error_service = nh.advertiseService("acknowledge_error", &EZN64_usb::acknowledge_error_callback, &gripper);
    ros::ServiceServer stop_service              = nh.advertiseService("stop", &EZN64_usb::stop_callback, &gripper);

    ros::spin();

    return(EXIT_SUCCESS);
}
