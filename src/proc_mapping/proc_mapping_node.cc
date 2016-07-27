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
#include <sonia_msgs/SemanticMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <functional>
#include "proc_mapping/config.h"
#include "proc_mapping/region_of_interest/rotated_rectangle.h"

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
      get_current_proc_tree_srv_(),
      get_proc_tree_list_srv_(),
      change_parameter_srv_(),
      send_map_srv_(),
      cs_(std::make_shared<CoordinateSystems>(nh_)),
      raw_map_(nh_, cs_),
      semantic_map_(cs_),
      map_interpreter_(nh_, "proc_trees", semantic_map_.GetObjectRegistery()) {
  map_pub_ = nh_->advertise<sonia_msgs::SemanticMap>("/proc_mapping/map", 100);
  markers_pub_ = nh_->advertise<visualization_msgs::MarkerArray>(
      "/proc_mapping/markers", 100);

  reset_map_sub_ = nh_->subscribe("reset_map", 100,
                                  &ProcMappingNode::ResetMapCallback, this);

  get_current_proc_tree_srv_ = nh_->advertiseService(
      "get_current_proc_tree", &ProcMappingNode::GetCurrentProcTreeCallback, this);

  get_proc_tree_list_srv_ = nh_->advertiseService(
      "get_proc_tree_list", &ProcMappingNode::GetProcTreeListCallback, this);

  change_parameter_srv_ = nh_->advertiseService(
      "change_parameter", &ProcMappingNode::ChangeParameterCallback, this);

  send_map_srv_ = nh_->advertiseService(
      "send_map", &ProcMappingNode::SendMapCallback, this);

  change_pt_srv_ = nh_->advertiseService(
      "change_proc_tree", &ProcMappingNode::ChangeProcTreeCallback, this);

  insert_rect_ROI_srv_ = nh_->advertiseService(
      "insert_rect_roi", &ProcMappingNode::InsertRectROICallback, this);

  insert_circle_ROI_srv_ = nh_->advertiseService(
      "insert_circle_roi", &ProcMappingNode::InsertCircleROICallback, this);

  raw_map_.Attach(map_interpreter_);
  map_interpreter_.Attach(semantic_map_);
}

//------------------------------------------------------------------------------
//
ProcMappingNode::~ProcMappingNode() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ProcMappingNode::ResetMapCallback(
    const sonia_msgs::ResetMap::ConstPtr &msg) {
  semantic_map_.ClearSemanticMap();
  raw_map_.ResetRawMap();
  semantic_map_.ResetSemanticMap();
  cs_->ResetPosition();
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::SendMapCallback(
    sonia_msgs::SendSemanticMap::Request &req,
    sonia_msgs::SendSemanticMap::Response &res) {
  ROS_INFO("Sending maps!");
  auto map_msg = semantic_map_.GenerateSemanticMapMessage();
  map_pub_.publish(map_msg);
  return true;
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::GetCurrentProcTreeCallback
    (sonia_msgs::GetCurrentProcTree::Request &req,
     sonia_msgs::GetCurrentProcTree::Response &res) {
  ProcTree::Ptr current_proc_tree;
  current_proc_tree = map_interpreter_.GetCurrentProcTree();
  res.current_proc_tree = current_proc_tree->BuildRosMessage();

  return true;
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::GetProcTreeListCallback(
    sonia_msgs::GetProcTreeList::Request &req,
    sonia_msgs::GetProcTreeList::Response &res) {
  std::vector<ProcTree::Ptr> proc_tree_list =
      map_interpreter_.GetProcTreeList();

  for (const auto pt : proc_tree_list) {
    res.proc_tree_list.push_back(pt->BuildRosMessage());
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::ChangeParameterCallback(
    sonia_msgs::ChangeParameter::Request &req,
    sonia_msgs::ChangeParameter::Response &resp) {
  ProcTree::Ptr pt = map_interpreter_.GetProcTree(req.proc_tree_name);
  ProcUnit::Ptr pu = pt->GetProcUnit(req.proc_unit_name);

  for (auto pup : pu->GetParameters()) {
    if (pup->GetName() == req.proc_unit_parameter.name) {
      pup->SetFromRosMessage(req.proc_unit_parameter);
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::ChangeProcTreeCallback(
    sonia_msgs::ChangeProcTree::Request &req,
    sonia_msgs::ChangeProcTree::Response &res) {
  ROS_INFO("Changing the proc tree");
  if (req.target == req.FAR_BUOYS) {
    map_interpreter_.SetDetectionMode(DetectionMode::FAR_BUOYS);
    ROS_INFO("Changing to far buoys");
  } else if (req.target == req.BUOYS) {
    map_interpreter_.SetDetectionMode(DetectionMode::BUOYS);
    ROS_INFO("Changing to buoys");
  } else if (req.target == req.FENCE) {
    map_interpreter_.SetDetectionMode(DetectionMode::FENCE);
    ROS_INFO("Changing to fence");
  } else if (req.target == req.WALL) {
    map_interpreter_.SetDetectionMode(DetectionMode::WALL);
  } else {
    map_interpreter_.SetDetectionMode(DetectionMode::NONE);
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::InsertRectROICallback(
    sonia_msgs::InsertRectROI::Request &req,
    sonia_msgs::InsertRectROI::Response &res) {
  // Todo: Reimplement this method
  // auto roi = std::make_shared<RotatedRectangle>(req.name, rect_point);
  // semantic_map_.InsertRegionOfInterest(std::move(roi));
  return true;
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::InsertCircleROICallback(
    sonia_msgs::InsertCircleROI::Request &req,
    sonia_msgs::InsertCircleROI::Response &res) {
  // Todo Implement the Circle ROI logic
  return true;
}

//------------------------------------------------------------------------------
//
void ProcMappingNode::Spin() {
  while (ros::ok()) {
    if (semantic_map_.IsNewDataAvailable()) {
      auto map_msg = semantic_map_.GenerateSemanticMapMessage();
      map_pub_.publish(map_msg);
      auto markers = semantic_map_.GenerateVisualizationMessage();
      markers_pub_.publish(markers);
    }

#ifdef DEBUG
    semantic_map_.PrintMap();
#endif

    ros::spinOnce();
  }
}

}  // namespace proc_mapping
