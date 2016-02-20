/**
 * \file	proc_mapping_node.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
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

#include <memory>
#include <vector>
#include <lib_atlas/macros.h>
#include <ros/node_handle.h>
#include "proc_mapping/object_mapper.h"

namespace proc_mapping {

class ProcMappingNode {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ProcMappingNode>;
  using ConstPtr = std::shared_ptr<const ProcMappingNode>;
  using PtrList = std::vector<ProcMappingNode::Ptr>;
  using ConstPtrList = std::vector<ProcMappingNode::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ProcMappingNode(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  ~ProcMappingNode() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Publisher semantic_map_pub_;

  ObjectMapper mapper_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_MAPPING_NODE_H_
