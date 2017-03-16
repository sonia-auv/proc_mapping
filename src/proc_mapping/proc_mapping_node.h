/**
 * \file	proc_mapping_node.h
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

#ifndef PROC_MAPPING_PROC_MAPPING_NODE_H_
#define PROC_MAPPING_PROC_MAPPING_NODE_H_

#include <lib_atlas/macros.h>
#include <ros/node_handle.h>
//#include <sonia_msgs/ResetMap.h>
#include <memory>
#include <vector>
#include "proc_mapping/config.h"
#include "proc_mapping/sonar/SonarMapper.h"

namespace proc_mapping {

class ProcMappingNode {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ProcMappingNode(const ros::NodeHandlePtr &nh);

  ~ProcMappingNode();

  /// Taking care of the spinning of the ROS thread.
  /// Each iteration of the loop, this will take the objects in the object
  /// registery, empty it and publish the objects.
  void Spin();

//  void ResetMapCallback(const sonia_msgs::ResetMap::ConstPtr &msg);



 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Publisher map_pub_;
  ros::Publisher markers_pub_;
  ros::Subscriber reset_map_sub_;

  SubmarinePosition submarine_position_;

  SonarMapper sonar_mapper_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_MAPPING_NODE_H_
