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

#include "proc_mapping/semantic_map.h"
#include "proc_mapping/interpreter/object_registery.h"
#include "proc_mapping/map_objects/buoy.h"
#include "proc_mapping/map_objects/fence.h"
#include "proc_mapping/map_objects/map_object.h"
#include "proc_mapping/map_objects/wall.h"
#include "proc_mapping/region_of_interest/contour.h"
#include "proc_mapping/region_of_interest/ellipse.h"
#include "proc_mapping/region_of_interest/rotated_rectangle.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
SemanticMap::SemanticMap(const RawMap::Ptr &raw_map)
    : raw_map_(raw_map),
      map_objects_({}),
      rois_{},
      new_objects_available_(false) {
  InsertRegionOfInterest("regions_of_interest.yaml");
}

//------------------------------------------------------------------------------
//
SemanticMap::~SemanticMap() = default;

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void SemanticMap::OnSubjectNotify(atlas::Subject<> &subject) {
  auto map_objects = ObjectRegistery::GetInstance().GetAllMapObject();
  ObjectRegistery::GetInstance().ClearRegistery();

  std::vector<Buoy *> buoys;
  for (auto &object : map_objects) {
    if (dynamic_cast<Buoy *>(object.get())) {
      buoys.push_back(dynamic_cast<Buoy *>(object.get()));
    } else if (dynamic_cast<Fence *>(object.get())) {
      // Todo: How to treat fences objects ?
    } else if (dynamic_cast<Wall *>(object.get())) {
      // Todo: How to treat walls ?
    } else {
      ROS_WARN("The map object is not recognized.");
    }
  }

  GetMetaDataForBuoys(std::move(buoys));
}

//------------------------------------------------------------------------------
//
void SemanticMap::GetMetaDataForBuoys(std::vector<Buoy *> &&map_objects) {
  std::lock_guard<std::mutex> guard(object_mutex_);

}

//------------------------------------------------------------------------------
//
const std::vector<SemanticMap::MapObjectsType> &SemanticMap::GetMapObjects() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  new_objects_available_ = false;
  return map_objects_;
}

//------------------------------------------------------------------------------
//
const std::vector<SemanticMap::RegionOfInterestType>
    &SemanticMap::GetRegionOfInterest() const {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return rois_;
}

//------------------------------------------------------------------------------
//
bool SemanticMap::IsNewDataAvailable() const {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return new_objects_available_;
}

//------------------------------------------------------------------------------
//
void SemanticMap::ClearMapObjects() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  map_objects_.clear();
  trigged_keypoints_.clear();
  new_objects_available_ = true;
}

//------------------------------------------------------------------------------
//
SemanticMap::RegionOfInterestType SemanticMap::RegionOfInterestFactory(
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
      rois_.push_back(roi);
    }
  }
}

}  // namespace proc_mapping
