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

#include <lib_atlas/macros.h>
#include <ros/node_handle.h>
#include <sonia_msgs/ChangeProcTree.h>
#include <sonia_msgs/InsertRectROI.h>
#include <sonia_msgs/InsertCircleROI.h>
#include <sonia_msgs/ResetOdometry.h>
#include <sonia_msgs/SendSemanticMap.h>
#include <memory>
#include <vector>
#include "proc_mapping/config.h"
#include "proc_mapping/interpreter/map_interpreter.h"
#include "proc_mapping/raw_map.h"
#include "proc_mapping/semantic_map.h"

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

  explicit ProcMappingNode(const ros::NodeHandlePtr &nh);

  ~ProcMappingNode();

  /// Taking care of the spinning of the ROS thread.
  /// Each iteration of the loop, this will take the objects in the object
  /// registery, empty it and publish the objects.
  void Spin();

  void ResetOdometryCallback(const sonia_msgs::ResetOdometry::ConstPtr &msg);

  bool SendMapCallback(sonia_msgs::SendSemanticMap::Request &req,
                       sonia_msgs::SendSemanticMap::Response &res);

  bool ChangeProcTreeCallback(sonia_msgs::ChangeProcTree::Request &req,
                              sonia_msgs::ChangeProcTree::Response &res);

  bool InsertRectROICallback(sonia_msgs::InsertRectROI::Request &req,
                             sonia_msgs::InsertRectROI::Response &res);

  bool InsertCircleROICallback(sonia_msgs::InsertCircleROI::Request &req,
                               sonia_msgs::InsertCircleROI::Response &res);

 private:

  void InsertCircleROI(std::string name, cv::Point2d center, int radius);

  void InsertRectROI(std::string name, cv::Rect rect);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Publisher map_pub_;
  ros::Subscriber reset_odom_sub_;
  ros::ServiceServer send_map_srv_;
  ros::ServiceServer change_pt_srv_;
  ros::ServiceServer insert_rect_ROI_srv_;
  ros::ServiceServer insert_circle_ROI_srv_;

  RawMap raw_map_;
  MapInterpreter map_interpreter_;
  SemanticMap semantic_map_;

#ifdef DEBUG
  /// This is a map used to display the submarine and objects detected in the
  /// semantic map.
  cv::Mat map_;
#endif
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_MAPPING_NODE_H_
