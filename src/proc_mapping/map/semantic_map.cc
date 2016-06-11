/**
 * \file	semantic_map.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	31/05/2016
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

#include "proc_mapping/map/semantic_map.h"
#include "proc_mapping/config.h"
#include "proc_mapping/map/object_registery.h"
#include "proc_mapping/map_objects/buoy.h"
#include "proc_mapping/map_objects/fence.h"
#include "proc_mapping/map_objects/map_object.h"
#include "proc_mapping/region_of_interest/contour.h"
#include "proc_mapping/region_of_interest/ellipse.h"
#include "proc_mapping/region_of_interest/rotated_rectangle.h"
#include "proc_mapping/region_of_interest/wall.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
SemanticMap::SemanticMap(const CoordinateSystems::Ptr &cs)
    : cs_(cs),
      object_registery_(),
#ifdef DEBUG
      display_map_(),
#endif
      new_objects_available_(false) {
  InsertRegionOfInterest("regions_of_interest.yaml");

#ifdef DEBUG
  display_map_ = cv::Mat(800, 800, CV_8UC1);
  display_map_.setTo(cv::Scalar(0));
#endif
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void SemanticMap::OnSubjectNotify(atlas::Subject<> &subject) {
  auto map_objects = object_registery_.GetAllMapObject();
  object_registery_.ClearRegistery();

  for (auto &object : map_objects) {
    if (dynamic_cast<Buoy *>(object.get())) {
#ifdef DEBUG
      object->DrawToMap(display_map_,
                        cs_->GetConvertionFunction<cv::Point2d>());
#endif
    } else if (dynamic_cast<Fence *>(object.get())) {
      // Todo: How to treat fences objects ?
    } else if (dynamic_cast<Wall *>(object.get())) {
      // Todo: How to treat walls ?
    } else {
      ROS_WARN("The map object is not recognized.");
    }
  }
}

//------------------------------------------------------------------------------
//
const std::vector<MapObject::Ptr> &SemanticMap::GetMapObjects() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  new_objects_available_ = false;
  return object_registery_.GetAllMapObject();
}

//------------------------------------------------------------------------------
//
const std::vector<RegionOfInterest::Ptr> &SemanticMap::GetRegionOfInterest()
    const {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return object_registery_.GetAllRegionOfInterest();
}

//------------------------------------------------------------------------------
//
bool SemanticMap::IsNewDataAvailable() const {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return new_objects_available_;
}

//------------------------------------------------------------------------------
//
void SemanticMap::ClearSemanticMap() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  object_registery_.ClearRegistery();
  new_objects_available_ = true;
}

//------------------------------------------------------------------------------
//
RegionOfInterest::Ptr SemanticMap::RegionOfInterestFactory(
    const YAML::Node &node) const {
  assert(node["roi_type"]);
  auto roi_type = node["roi_type"].as<std::string>();

  RegionOfInterest *r = nullptr;
  if (roi_type == "rotated_rectangle") {
    r = new RotatedRectangle(node);
  } else if (roi_type == "contour") {
    r = new Contour(node);
  } else if (roi_type == "ellipse") {
    r = new Ellipse(node);
  }

  if (r) {
    return RegionOfInterest::Ptr{r};
  }

  return nullptr;
}

//------------------------------------------------------------------------------
//
void SemanticMap::InsertRegionOfInterest(
    const std::string &proc_tree_file_name) {
  YAML::Node node = YAML::LoadFile(kConfigFilePath + proc_tree_file_name);

  assert(node["regions_of_interest"]);
  auto regions_of_interests = node["regions_of_interest"];
  assert(regions_of_interests.Type() == YAML::NodeType::Sequence);

  std::lock_guard<std::mutex> guard(object_mutex_);
  for (size_t i = 0; i < regions_of_interests.size(); ++i) {
    auto roi = RegionOfInterestFactory(regions_of_interests[i]);
    if (roi) {
      object_registery_.AddRegionOfInterest(roi);
    }
  }
}

//------------------------------------------------------------------------------
//
sonia_msgs::SemanticMap SemanticMap::GenerateSemanticMapMessage() const {
  return sonia_msgs::SemanticMap();
}

//------------------------------------------------------------------------------
//
visualization_msgs::MarkerArray SemanticMap::GenerateVisualizationMessage()
    const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "proc_mapping";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = cs_->GetSub().position.x;
  marker.pose.position.y = cs_->GetSub().position.y;
  marker.pose.position.z = cs_->GetSub().position.z;
  marker.pose.orientation.x = cs_->GetSub().roll;
  marker.pose.orientation.y = cs_->GetSub().pitch;
  marker.pose.orientation.z = cs_->GetSub().yaw;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
                         //  marker.color.r = 0.0;
                         //  marker.color.g = 1.0;
                         //  marker.color.b = 0.0;
                         // only if using a MESH_RESOURCE marker type:
  marker.mesh_resource = "package://proc_mapping/pr2_description/SUB.dae";

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  return marker_array;
}

//------------------------------------------------------------------------------
//
ObjectRegistery::Ptr SemanticMap::GetObjectRegistery() {
  return ObjectRegistery::Ptr{&object_registery_};
}

//------------------------------------------------------------------------------
//
void SemanticMap::ResetSemanticMap() { display_map_.setTo(cv::Scalar(0)); }

#ifdef DEBUG
//------------------------------------------------------------------------------
//
void SemanticMap::PrintMap() {
  cv::Point2d offset = cs_->GetPositionOffset();
  auto sub = cv::Point2d(cs_->GetSub().position.x, cs_->GetSub().position.y);
  sub += offset;

  auto pixel = cs_->GetPixel();
  for (int i = 0; i < pixel.height; i += pixel.m_to_pixel) {
    cv::line(display_map_, cv::Point2d(i, 0), cv::Point2d(i, pixel.height),
             cv::Scalar(255));
    cv::line(display_map_, cv::Point2d(0, i), cv::Point2d(pixel.width, i),
             cv::Scalar(255));
  }

  for (const auto &roi : GetRegionOfInterest()) {
    roi->DrawRegion(display_map_, cs_->GetConvertionFunction<cv::Point2d>());
  }

  sub = cs_->WorldToPixelCoordinates(sub);
  sub.y = (pixel.height / 2) - sub.y + (pixel.height / 2);

  cv::circle(display_map_, sub, 5, cv::Scalar(255), -1);
  cv::imshow("Semantic Map", display_map_);
  cv::circle(display_map_, sub, 5, cv::Scalar(0), -1);

  cv::waitKey(1);
}
#endif

}  // namespace proc_mapping
