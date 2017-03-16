/**
 * \file	SonarMapper.h
 * \author	Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date	06/02/2016
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

#ifndef PROC_MAPPING_RAW_MAP_H_
#define PROC_MAPPING_RAW_MAP_H_

#include <opencv/cv.h>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "proc_mapping/general/AsyncImagePublisher.h"
#include "proc_mapping/general/BaseObjectMapperInterface.h"
#include "proc_mapping/general/SubmarinePosition.h"

namespace proc_mapping {

class SonarMapper : public BaseObjectMapperInterface
{
public:
  SonarMapper(const SubmarinePosition &submarine_position);

  // Override BaseObjectMapperInterface
  void GetMapObject(MapObjectVector &list) override;
  void ResetMapper() override;

private:

private:
  cv::Mat sonar_map_;
  MapObjectVector object_list_;
  const SubmarinePosition &submarine_position_;
};

inline void SonarMapper::GetMapObject(MapObjectVector &list)
{
  list.clear();
  std::copy(object_list_.begin(), object_list_.end(), list.begin());
}

inline void SonarMapper::ResetMapper()
{
  sonar_map_.setTo(0);
  object_list_.clear();
}


}  // namespace proc_mapping

#endif  // PROC_MAPPING_RAW_MAP_H_
