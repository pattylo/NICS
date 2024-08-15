/*
    This file is part of NICS - the non-robocentric dynamic landing system for quadrotor

    NICS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    NICS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NICS.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file vdrse_lib.cpp
 * \date 16/08/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#include "led_rse/vdrse_lib.h"

nics::VdrseLib::VdrseLib(ros::NodeHandle& nh)
{
	ROS_INFO("VDRSE Nodelet Initiated...");

    doALOTofConfigs(nh);
}

nics::VdrseLib::~VdrseLib()
{
    
}