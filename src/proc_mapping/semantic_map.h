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
#include "proc_mapping/raw_map.h"

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

  using MapObjectsType = sonia_msgs::MapObject;

  struct Keypoint {
    cv::KeyPoint trigged_keypoint;
    cv::Rect bounding_box;
    bool is_object_send;
  };

  enum class DetectionMode {
    NO_MODE = 0,
    BUOYS,
    FENCE,
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit SemanticMap(const RawMap::Ptr &raw_map);

  virtual ~SemanticMap();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void OnSubjectNotify(atlas::Subject<> &) override;

  const std::vector<MapObjectsType> &GetMapObjects();

  bool IsNewDataAvailable() const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /// Passing rvalue for optimization purpose, we are just using it as a stream
  /// line of functions anyway.
  void GetMetaDataForBuoys(std::vector<cv::KeyPoint> &&);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  RawMap::Ptr raw_map_;

  std::vector<MapObjectsType> map_objects_;

  DetectionMode mode_;

  bool new_objects_available_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_SEMANTIC_MAP_H_
