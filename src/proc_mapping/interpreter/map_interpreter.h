/**
 * \file	raw_map_interpreter.h
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

#ifndef PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_
#define PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_

#include <lib_atlas/macros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <array>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <vector>
#include "proc_mapping/interpreter/data_interpreter.h"
#include "proc_mapping/raw_map.h"

namespace proc_mapping {

class MapInterpreter : public DataInterpreter<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MapInterpreter>;
  using ConstPtr = std::shared_ptr<const MapInterpreter>;
  using PtrList = std::vector<MapInterpreter::Ptr>;
  using ConstPtrList = std::vector<MapInterpreter::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit MapInterpreter(const ros::NodeHandlePtr &nh);

  virtual ~MapInterpreter();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /// The method will update the latest data in the DataInterpreter.
  /// This will run the whole processing chain as the method SetNewData
  /// start it.
  void OnSubjectNotify(atlas::Subject<cv::Mat> &subject, cv::Mat args) override;

  std::vector<std::shared_ptr<sonia_msgs::MapObject>> GetMapObjects() const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  RawMap raw_map_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_