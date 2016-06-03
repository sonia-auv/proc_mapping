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
      raw_map_(nh_),
      map_interpreter_(nh_, "proc_trees"),
      semantic_map_(std::shared_ptr<RawMap>(&raw_map_)) {
  map_pub_ =
      nh_->advertise<sonia_msgs::MapObject>("/proc_mapping/objects", 100);

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
void ProcMappingNode::Spin() {
  while (ros::ok()) {
    cv::Point2d offset = raw_map_.GetPositionOffset();

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
#endif

    cv::waitKey(1);

    ros::spinOnce();
  }
}

}  // namespace proc_mapping
