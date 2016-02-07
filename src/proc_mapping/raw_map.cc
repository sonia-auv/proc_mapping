/**
 * \file	can_device.cc
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

#include "proc_mapping/raw_map.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
pcl::PointCloud<pcl::PointXYZ> RawMap::MatToPointXYZ(const cv::Mat &m) noexcept {
  pcl::PointCloud<pcl::PointXYZ> point_cloud;

  for (int i = 0; i < m.cols; i++) {
    pcl::PointXYZ p;
    p.x = m.at<float>(0, i);
    p.y = m.at<float>(1, i);
    p.z = m.at<float>(2, i);
    point_cloud.points.push_back(p);
  }

  point_cloud.width = static_cast<int>(point_cloud.points.size());
  point_cloud.height = 1;
  return point_cloud;
}

//------------------------------------------------------------------------------
//
cv::Mat RawMap::PointXYZToMat(
    const pcl::PointCloud<pcl::PointXYZ> &point_cloud) noexcept {
  cv::Mat OpenCVPointCloud(3, static_cast<int>(point_cloud.points.size()),
                           CV_64FC1);
  for (uint64_t i = 0; i < point_cloud.points.size(); i++) {
    OpenCVPointCloud.at<double>(0, static_cast<int>(i)) =
        point_cloud.points.at(i).x;
    OpenCVPointCloud.at<double>(1, static_cast<int>(i)) =
        point_cloud.points.at(i).y;
    OpenCVPointCloud.at<double>(2, static_cast<int>(i)) =
        point_cloud.points.at(i).z;
  }

  return OpenCVPointCloud;
}

}  // namespace proc_mapping
