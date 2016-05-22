/**
 * \file	raw_map_interpreter.cc
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

#include "proc_mapping/interpreter/map_interpreter.h"
#include <tf/transform_datatypes.h>
#include "proc_mapping/proc_unit/gaussian_blur.h"
#include "proc_mapping/proc_unit/threshold.h"
#include "proc_mapping/proc_unit/dilate.h"
#include "proc_mapping/proc_unit/blob_detector.h"
#include "proc_mapping/proc_unit/pattern_detection.h"

namespace proc_mapping {
// - Blurr params
int gaussian_kernelSize = 5;
// - Dilate params
int dilate_kernelSize_x = 5;
int dilate_kernelSize_y = 5;
// - Blob detector params
int blob_minArea = 30;
int blob_maxArea = 300;
bool blob_filterByConvexity = false;
double blob_minConvexity = 0.1;
double blob_maxConvexity = 0.8;
bool blob_filterByInertia = false;
double blob_minInertiaRatio = 0;
double blob_maxInertiaRatio = 0.5;
//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MapInterpreter::MapInterpreter(const ros::NodeHandlePtr &nh)
        : DataInterpreter<cv::Mat>(nh), nh_(nh), raw_map_(nh_) {
            Observe(raw_map_);
  ProcUnit<cv::Mat>::Ptr pu1{new GaussianBlur(gaussian_kernelSize)};
  AddProcUnit(std::move(pu1));
  ProcUnit<cv::Mat>::Ptr pu2{new Threshold()};
  AddProcUnit(std::move(pu2));
  ProcUnit<cv::Mat>::Ptr pu3{new Dilate(dilate_kernelSize_x, dilate_kernelSize_y)};
  AddProcUnit(std::move(pu3));
  ProcUnit<cv::Mat>::Ptr pu4{new BlobDetector(blob_minArea, blob_maxArea, blob_filterByConvexity,
                                              blob_minConvexity, blob_maxConvexity, blob_filterByInertia,
                                              blob_minInertiaRatio, blob_maxInertiaRatio)};
  AddProcUnit(std::move(pu4));

/*
  // This is not a proc unit that is going to be used, but let's keep it
  // for demo purpose for now, we will delete it once every thing works with
  // the other algos.
  ProcUnit<cv::Mat>::Ptr test_pu{new PatternDetection(nh)};
  AddProcUnit(std::move(test_pu));
   */
}

//------------------------------------------------------------------------------
//
MapInterpreter::~MapInterpreter() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void MapInterpreter::OnSubjectNotify(atlas::Subject<cv::Mat> &subject,
                                     cv::Mat args) {
  /// We want to recreate a new map everytime so the processed map does not
  /// interfer with the real one.
  cv::Mat new_data;
  args.copyTo(new_data);
  SetNewData(new_data);
  ProcessData();
}

//------------------------------------------------------------------------------
//
std::vector<std::shared_ptr<sonia_msgs::MapObject>>
    MapInterpreter::GetMapObjects() const {
  auto map_objects = ObjectRegistery::GetInstance().GetAllMapObject();
  ObjectRegistery::GetInstance().ClearRegistery();
  std::vector<std::shared_ptr<sonia_msgs::MapObject>> map_obj_msgs;
  for (const auto & object : map_objects) {
    auto msg = std::make_shared<sonia_msgs::MapObject>();
    cv::Point2d offset = raw_map_.GetPositionOffset();
    cv::Point2i object_coordinate(object.pose.x, object.pose.y);
    auto world_point = raw_map_.PixelToWorldCoordinates(object_coordinate);
    msg->name = object.name;
    msg->pose.x = world_point.x - offset.x;
    msg->pose.y = world_point.y - offset.y;
    msg->pose.theta = raw_map_.GetSubMarineYaw();
    /// Casting to rvalue, we loose the variable at the end of scope.
    map_obj_msgs.push_back(std::move(msg));
  }
  return map_obj_msgs;
}


}  // namespace proc_mapping
