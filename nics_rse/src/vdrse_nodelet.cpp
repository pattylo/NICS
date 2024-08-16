/*
    This file is part of NICS - a non-inertial control system

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
 * \file ledvo_nodelet.h
 * \date 16/08/2024
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief nodelet cpp
*/

#include "nics_rse/vdrse_nodelet.h"

void nics::VdrseNodelet::onInit()
{
    ROS_CYAN_STREAM("STARTO!");
    ros::NodeHandle& nh = getMTNodeHandle();
    vdrselib_ptr = std::make_shared<VdrseLib>(getMTNodeHandle());
    
    return;
}