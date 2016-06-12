/**
 * \file	buoy.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/06/2016
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

#ifndef PROC_MAPPING_MAP_OBJECT_MAP_OBJECT_H_
#define PROC_MAPPING_MAP_OBJECT_MAP_OBJECT_H_

#include <highgui.h>
#include <sonia_msgs/MapObject.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include <vector>
#include "proc_mapping/map_objects/map_object.h"

namespace proc_mapping {

class MapObject {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MapObject>;
  using ConstPtr = std::shared_ptr<const MapObject>;
  using PtrList = std::vector<MapObject::Ptr>;
  using ConstPtrList = std::vector<MapObject::ConstPtr>;

  struct Pose {
    double x;
    double y;
    double z;
  };

  struct Orientation {
    double yaw;
    double pitch;
    double roll;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit MapObject();
  virtual ~MapObject();

  //==========================================================================
  // P U B L I C   M E T H O D S

  const std::string &GetName() const;
  void SetName(const std::string &name);

  const Pose &GetPose() const;
  void SetPose(const Pose &pose);

  const Orientation &GetOrientation() const;
  void SetOrientation(const Orientation &orientation);

  const cv::KeyPoint &GetCvKeyPoint() const;
  void SetCvKeyPoint(const cv::KeyPoint &key_point);

  const cv::Rect &GetBoundingBox() const;
  void SetBoundingBox(const cv::Rect &box);

  void AddWeight(uint8_t to_add);
  void RemoveWeight(uint8_t to_remove);

  sonia_msgs::MapObject GenerateToMapObjectMessge() const;
  virtual visualization_msgs::Marker GenerateVisualizationMarker(
      int id) const = 0;

  virtual void DrawToMap(cv::Mat,
                         const std::function<cv::Point2i(const cv::Point2d &p)>
                             &convert) const = 0;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  virtual uint8_t GetMessageObjectType() const = 0;

  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  std::string name_;
  Pose pose_;
  Orientation orientation_;

  cv::KeyPoint trigged_keypoint_;
  cv::Rect bounding_box_;
  uint8_t weight_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_MAP_OBJECT_MAP_OBJECT_H_
