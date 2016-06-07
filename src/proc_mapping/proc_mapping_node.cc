/**
 * \file	proc_mapping_node.cc
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

#include "proc_mapping/proc_mapping_node.h"
#include <sonia_msgs/SemanticMap.h>
#include "proc_mapping/config.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProcMappingNode::ProcMappingNode(const ros::NodeHandlePtr &nh)
    : nh_(nh),
      map_pub_(),
      reset_odom_sub_(),
      send_map_srv_(),
      raw_map_(nh_),
      map_interpreter_(nh_, "proc_trees"),
      semantic_map_(std::shared_ptr<RawMap>(&raw_map_)) {
  map_pub_ = nh_->advertise<sonia_msgs::SemanticMap>("/proc_mapping/map", 100);

  reset_odom_sub_ =
      nh_->subscribe("/proc_navigation/reset_odometry", 100,
                     &ProcMappingNode::ResetOdometryCallback, this);
  send_map_srv_ = nh_->advertiseService(
      "send_map", &ProcMappingNode::SendMapCallback, this);

  change_pt_srv_ = nh_->advertiseService(
      "change_proc_tree", &ProcMappingNode::ChangeProcTreeCallback, this);

  insert_rect_ROI_srv_ = nh_->advertiseService(
      "insert_rect_ROI", &ProcMappingNode::InsertRectROICallback, this);

  insert_circle_ROI_srv_ = nh_->advertiseService(
      "insert_circle_ROI", &ProcMappingNode::InsertCircleROICallback, this);

  raw_map_.Attach(map_interpreter_);
  map_interpreter_.Attach(semantic_map_);

#ifdef DEBUG
  map_ = cv::Mat(800, 800, CV_8UC1);
  map_.setTo(cv::Scalar(0));
#endif
}

//------------------------------------------------------------------------------
//
ProcMappingNode::~ProcMappingNode() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ProcMappingNode::ResetOdometryCallback(
    const sonia_msgs::ResetOdometry::ConstPtr &msg) {
  semantic_map_.ClearMapObjects();
  map_.setTo(cv::Scalar(0));
  raw_map_.ResetRawMap();
  raw_map_.ResetPosition();
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::SendMapCallback(
    sonia_msgs::SendSemanticMap::Request &req,
    sonia_msgs::SendSemanticMap::Response &res) {
  sonia_msgs::SemanticMap map_msg;
  for (const auto &obj : semantic_map_.GetMapObjects()) {
    map_msg.objects.push_back(obj);
  }
  map_pub_.publish(map_msg);
  return true;
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::ChangeProcTreeCallback(
    sonia_msgs::ChangeProcTree::Request &req,
    sonia_msgs::ChangeProcTree::Response &res) {
  if (req.target == req.BUOYS) {
    map_interpreter_.SetDetectionMode(DetectionMode::BUOYS);
  } else if (req.target == req.FENCE) {
    map_interpreter_.SetDetectionMode(DetectionMode::FENCE);
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
  std::vector<cv::Point> rect_point;
  rect_point.push_back(raw_map_.WorldToPixelCoordinates(cv::Point2d(req.a.x, req.a.y)));
  rect_point.push_back(raw_map_.WorldToPixelCoordinates(cv::Point2d(req.b.x, req.b.y)));
  rect_point.push_back(raw_map_.WorldToPixelCoordinates(cv::Point2d(req.c.x, req.c.y)));
  rect_point.push_back(raw_map_.WorldToPixelCoordinates(cv::Point2d(req.d.x, req.d.y)));
  InsertRectROI(req.name, cv::boundingRect(rect_point));
  return true;
}

//------------------------------------------------------------------------------
//
bool ProcMappingNode::InsertCircleROICallback(
    sonia_msgs::InsertCircleROI::Request &req,
    sonia_msgs::InsertCircleROI::Response &res) {
  cv::Point2d center;
  center.x = req.center.x;
  center.y = req.center.y;
  InsertCircleROI(req.name, raw_map_.WorldToPixelCoordinates(center), req.radius);
  return true;
}

//------------------------------------------------------------------------------
//
void ProcMappingNode::InsertCircleROI(std::string name, cv::Point2d center, int radius) {
  cv::circle(map_, center, radius, cv::Scalar(255), 2);
}

//------------------------------------------------------------------------------
//
void ProcMappingNode::InsertRectROI(std::string name, cv::Rect rect) {
  cv::rectangle(map_, rect, cv::Scalar(255), 2);
}


//------------------------------------------------------------------------------
//
void ProcMappingNode::Spin() {
  while (ros::ok()) {
#ifdef DEBUG
    cv::Point2d offset = raw_map_.GetPositionOffset();
#endif

    if (semantic_map_.IsNewDataAvailable()) {
      sonia_msgs::SemanticMap map_msg;
      for (const auto &obj : semantic_map_.GetMapObjects()) {
        map_msg.objects.push_back(obj);
#ifdef DEBUG
        cv::Point2d object(obj.pose.x, obj.pose.y);
        object += offset;
        object = raw_map_.WorldToPixelCoordinates(object);
        cv::circle(map_, object, 10, cv::Scalar(255), -1);
#endif
      }
      map_pub_.publish(map_msg);
    }

#ifdef DEBUG
    cv::Point2d sub = raw_map_.GetSubMarinePosition();
    sub += offset;

    cv::line(map_, cv::Point2d(800 / 2, 800 / 2), cv::Point2d(800 / 2, 0),
             cv::Scalar(255));
    cv::line(map_, cv::Point2d(800 / 2, 800 / 2), cv::Point2d(800, 800 / 2),
             cv::Scalar(255));

    sub = raw_map_.WorldToPixelCoordinates(sub);
    sub.y = (800 / 2) - sub.y + (800 / 2);

    cv::circle(map_, sub, 5, cv::Scalar(255), -1);
    cv::imshow("Semantic Map", map_);
    cv::circle(map_, sub, 5, cv::Scalar(0), -1);

    cv::waitKey(1);
#endif

    ros::spinOnce();
  }
}

}  // namespace proc_mapping
