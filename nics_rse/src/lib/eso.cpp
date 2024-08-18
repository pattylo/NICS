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
 * \file eso.cpp
 * \date 16/08/2024
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief lib - configuration
*/

#include "nics_rse/vdrse_lib.h"

void nics::VdrseLib::set_dz()
{
    using namespace std;
    
    y_eso = pose_uav_inWorld_SE3.translation();

    dz_I = A_eso * z_I 
        + B_eso * u_input 
        + L_eso * (y_eso - C_eso * z_I);
    // cout<<dz_I<<endl<<endl;
}

void nics::VdrseLib::update_z()
{
    using namespace std;
    t_eso = ros::Time::now().toSec() - t_eso_prev;
    if(t_eso > 1)
    {   
        std::cout<<t_eso<<std::endl;
        patty::Debug("IN update_z");
    }
        
    z_I = z_I + t_eso * dz_I;
    t_eso_prev = ros::Time::now().toSec();

    // cout<<z_I.head(3)<<endl;
}

void nics::VdrseLib::get_gain()
{
    L_eso;
}

void nics::VdrseLib::pub_z()
{
    z_dist.header.frame_id = "world";
    z_dist.header.stamp = ros::Time::now();
    
    z_dist.point.x = z_I.tail(3).x();
    z_dist.point.y = z_I.tail(3).y();
    z_dist.point.z = z_I.tail(3).z();

    z_dist_pub.publish(z_dist); 
}