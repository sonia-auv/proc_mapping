/**
 * \file	matrix_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
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

#include <memory>
#include <vector>
#include <opencv/cv.h>
#include <pcl_ros/point_cloud.h>
#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/subject.h>

namespace proc_mapping {

class RawMap : atlas::Subject<const cv::Mat &> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RawMap>;
  using ConstPtr = std::shared_ptr<const RawMap>;
  using PtrList = std::vector<RawMap::Ptr>;
  using ConstPtrList = std::vector<RawMap::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  RawMap() ATLAS_NOEXCEPT;

  ~RawMap() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

  static pcl::PointCloud<pcl::PointXYZ> MatToPointXYZ(const cv::Mat &m)
      ATLAS_NOEXCEPT;

  static cv::Mat PointXYZToMat(
      const pcl::PointCloud<pcl::PointXYZ> &point_cloud) ATLAS_NOEXCEPT;
};

}  // namespace proc_mapping
