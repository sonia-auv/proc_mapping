/**
 * \file	semantic_map.h
 * \author	Francis Masse <francis.masse05@gmail.com>
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

#ifndef PROC_MAPPING_SEMANTIC_MAP_H_
#define PROC_MAPPING_SEMANTIC_MAP_H_

#include <lib_atlas/pattern/observer.h>
#include <opencv/cv.h>
#include <sonia_msgs/MapObject.h>
#include <sonia_msgs/SemanticMap.h>
#include <visualization_msgs/MarkerArray.h>
#include "proc_mapping/config.h"
#include "proc_mapping/map/coordinate_systems.h"
#include "proc_mapping/map/object_registery.h"
#include "proc_mapping/map_objects/buoy.h"
#include "proc_mapping/region_of_interest/region_of_interest.h"

namespace proc_mapping {

/// Inheriting a blank observer, just receiving a notification when a process
/// loop has been done (We don't want to know anything about the raw map or the
/// data interpreters)
class SemanticMap : public atlas::Observer<> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<SemanticMap>;
  using ConstPtr = std::shared_ptr<const SemanticMap>;
  using PtrList = std::vector<SemanticMap::Ptr>;
  using ConstPtrList = std::vector<SemanticMap::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit SemanticMap(const CoordinateSystems::Ptr &cs);

  ~SemanticMap() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void OnSubjectNotify(atlas::Subject<> &subject) override;

  /// Proxy method for that returns all the MapObject from the ObjectRegistery.
  const ObjectRegistery::MapObjectList &GetMapObjects();
  const ObjectRegistery::RegionOfInterestList &GetRegionOfInterest() const;
  void InsertRegionOfInterest(const std::string &proc_tree_file_name);
  void ClearSemanticMap();

  sonia_msgs::SemanticMap GenerateSemanticMapMessage() const;
  visualization_msgs::MarkerArray GenerateVisualizationMessage() const;

  bool IsNewDataAvailable() const;

  ObjectRegistery::Ptr GetObjectRegistery();

  void ResetSemanticMap();

#ifdef DEBUG
  void PrintMap();
#endif
 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /// Get the list of the regions of interest from the config file and
  /// instanciate all of them by sending them the appropriate YAML node.
  RegionOfInterest::Ptr RegionOfInterestFactory(const YAML::Node &node) const;

  visualization_msgs::Marker GenerateSubmarineMarker() const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  CoordinateSystems::Ptr cs_;
  ObjectRegistery object_registery_;

#ifdef DEBUG
  cv::Mat display_map_;
#endif

  bool new_objects_available_;
  mutable std::mutex object_mutex_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_SEMANTIC_MAP_H_
