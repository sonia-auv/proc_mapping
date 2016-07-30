/**
 * \file	semantic_map.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	31/05/2016
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
      new_objects_available_(false)
#ifdef DEBUG
      ,
      display_map_(),
      image_publisher_(kRosNodeName + "semantic_map")
#endif
{
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
  new_objects_available_ = true;

  for (auto &object : map_objects) {
    if (dynamic_cast<Buoy *>(object.get())) {
#ifdef DEBUG
      object->DrawToMap(display_map_);
#endif
    } else if (dynamic_cast<Fence *>(object.get())) {
#ifdef DEBUG
      object->DrawToMap(display_map_);
#endif
    } else if (dynamic_cast<Wall *>(object.get())) {
      // Todo: How to treat walls ?
    } else {
      ROS_WARN(
          "The semantic map received a new object that is not recognized.");
    }
  }
}

//------------------------------------------------------------------------------
//
const std::vector<MapObject::Ptr> &SemanticMap::GetMapObjects() {
  new_objects_available_ = false;
  return object_registery_.GetAllMapObject();
}

//------------------------------------------------------------------------------
//
const std::vector<RegionOfInterest::Ptr> &SemanticMap::GetRegionOfInterest()
    const {
  return object_registery_.GetAllRegionOfInterest();
}

//------------------------------------------------------------------------------
//
bool SemanticMap::IsNewDataAvailable() const { return new_objects_available_; }

//------------------------------------------------------------------------------
//
void SemanticMap::ClearSemanticMap() {
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
  //  assert(regions_of_interests.Type() == YAML::NodeType::Sequence);

  for (size_t i = 0; i < regions_of_interests.size(); ++i) {
    auto roi = RegionOfInterestFactory(regions_of_interests[i]);
    if (roi) {
      object_registery_.AddRegionOfInterest(roi);
    }
  }
}

//------------------------------------------------------------------------------
//
void SemanticMap::InsertRegionOfInterest(const RegionOfInterest::Ptr roi) {
  object_registery_.AddRegionOfInterest(roi);
}

//------------------------------------------------------------------------------
//
sonia_msgs::SemanticMap SemanticMap::GenerateSemanticMapMessage() {
  sonia_msgs::SemanticMap map_msg;
  for (const auto &obj : GetMapObjects()) {
    sonia_msgs::MapObject map_object =
        cs_->PixelToWorldCoordinates(obj->GenerateToMapObjectMessge());
    map_object.pose.x -= cs_->GetPositionOffset().x;
    map_object.pose.y -= cs_->GetPositionOffset().y;
    ROS_INFO_STREAM(
        "Detecting a BUOYS object at the position ["
            << map_object.pose.x << ";"
            << map_object.pose.y << "]");
    map_msg.objects.push_back(map_object);
  }
  return map_msg;
}

//------------------------------------------------------------------------------
//
visualization_msgs::MarkerArray SemanticMap::GenerateVisualizationMessage() {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(GenerateSubmarineMarker());

  auto map_objects = object_registery_.GetAllMapObject();

  int i = 1;
  for (const auto &obj : map_objects) {
    auto marker = obj->GenerateVisualizationMarker(i);
    marker.pose.position.x =
        cs_->PixelToWorldCoordinates(marker.pose.position.x);
    marker.pose.position.y =
        cs_->PixelToWorldCoordinates(marker.pose.position.y);
    marker.pose.position.z = 0;
    marker_array.markers.push_back(marker);
    ++i;
  }

  return marker_array;
}

//------------------------------------------------------------------------------
//
visualization_msgs::Marker SemanticMap::GenerateSubmarineMarker() const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "NED";
  marker.header.stamp = ros::Time();
  marker.ns = "proc_mapping";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = cs_->GetSub().position.x;
  marker.pose.position.y = cs_->GetSub().position.y;
  marker.pose.position.z = cs_->GetSub().position.z;
  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;
  marker.color.a = 1.0;
  marker.mesh_resource = "package://proc_mapping/meshes/sub.stl";

  Eigen::Matrix3d rotation = cs_->GetSub().orientation.toRotationMatrix();
  Eigen::Vector3d euler_vec = rotation.eulerAngles(0, 1, 2);

  double roll = fmod(euler_vec.x() + M_PI / 2, 2 * M_PI);
  double pitch = euler_vec.y();
  double yaw = fmod(euler_vec.z() + M_PI, 2 * M_PI);

  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  q.normalize();

  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  return marker;
}

//------------------------------------------------------------------------------
//
ObjectRegistery::Ptr SemanticMap::GetObjectRegistery() {
  return ObjectRegistery::Ptr{&object_registery_};
}

//------------------------------------------------------------------------------
//
void SemanticMap::ResetSemanticMap() {
#ifdef DEBUG
  display_map_.setTo(cv::Scalar(0));
#endif
}

#ifdef DEBUG
//------------------------------------------------------------------------------
//
void SemanticMap::PrintMap() {
  auto sub = cv::Point2d(cs_->GetSub().position.x, cs_->GetSub().position.y);
  sub += cs_->GetPositionOffset();

  // Create sub heading
  cv::Point2d heading(cos(cs_->GetSub().yaw), sin(cs_->GetSub().yaw));
//  heading += cs_->GetPositionOffset();

  auto pixel = cs_->GetPixel();
  for (int i = 0; i < pixel.height; i += pixel.m_to_pixel) {
    cv::line(display_map_, cv::Point2d(i, 0), cv::Point2d(i, pixel.height),
             cv::Scalar(255));
    cv::line(display_map_, cv::Point2d(0, i), cv::Point2d(pixel.width, i),
             cv::Scalar(255));
  }

  for (const auto &roi : GetRegionOfInterest()) {
    roi->DrawRegion(display_map_, cs_->GetWorldToPixelFunction<cv::Point2d>());
  }

  sub = cs_->WorldToPixelCoordinates(sub);
  heading = cs_->WorldToPixelCoordinates(heading);

  // Draw the sub in the map
  cv::circle(display_map_, sub, 5, cv::Scalar(255), -1);
  // Draw heading
  cv::line(display_map_,heading, sub, cv::Scalar::all(255));

  // Draw the origin in the map
  auto origin = cs_->GetWorld().origin;
  origin = cs_->WorldToPixelCoordinates(origin);
  cv::circle(display_map_, origin, 8, cv::Scalar(255), -1);

  // To fit in OpenCv coordinate system, we have to made a rotation of
  // 90 degrees on the display map
  cv::Point2f src_center(display_map_.cols / 2.0f, display_map_.rows / 2.0f);
  cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
  cv::Mat dst;
  cv::warpAffine(display_map_, dst, rot_mat, display_map_.size());

  cvtColor(dst, dst, CV_GRAY2RGB);
  image_publisher_.Publish(dst);

  // Remove the old sub position
  cv::circle(display_map_, sub, 5, cv::Scalar(0), -1);
  // Remove old heading
  cv::line(display_map_,heading, sub, cv::Scalar::all(0));
}
#endif

}  // namespace proc_mapping
