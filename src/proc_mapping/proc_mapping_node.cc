/**
 * \file	proc_mapping_node.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "proc_mapping/proc_mapping_node.h"
#include <visualization_msgs/MarkerArray.h>
#include <functional>
#include "proc_mapping/config.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProcMappingNode::ProcMappingNode(const ros::NodeHandlePtr &nh)
    : nh_(nh),
      map_pub_(),
      markers_pub_(),
      reset_map_sub_(),
      submarine_position_(),
      sonar_mapper_(submarine_position_)
{
//  map_pub_ = nh_->advertise<sonia_msgs::SemanticMap>("/proc_mapping/map", 100);
  markers_pub_ = nh_->advertise<visualization_msgs::MarkerArray>(
      "/proc_mapping/markers", 100);

//  reset_map_sub_ = nh_->subscribe("reset_map", 100,
//                                  &ProcMappingNode::ResetMapCallback, this);

}

//------------------------------------------------------------------------------
//
ProcMappingNode::~ProcMappingNode() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
//void ProcMappingNode::ResetMapCallback(
//    const sonia_msgs::ResetMap::ConstPtr &msg) {
//  ROS_INFO("Resetting the mappers object.");
//  sonar_mapper_.ResetMapper();
//}

//------------------------------------------------------------------------------
//
void ProcMappingNode::Spin() {
  ros::Rate r(15);  // 15 hz
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

}  // namespace proc_mapping
