/**
 * \file	SonarMapper.cc
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


#include "proc_mapping/sonar/SonarMapper.h"

namespace proc_mapping {

SonarMapper::SonarMapper(const SubmarinePosition &submarine_position)
    : sonar_map_(MAP_HEIGTH_METER*NB_PIXEL_BY_METER, MAP_WIDTH_METER*NB_PIXEL_BY_METER, CV_8UC1),
      object_list_(0),
      submarine_position_(submarine_position),
      scanline_count_(0),
      image_publisher_("sonar_map")
{
  std::string topic_name ("/provider_sonar/point_cloud2" );
  auto hdl = ros::NodeHandle("~");
  scanline_subscriber_ = hdl.subscribe(topic_name, 100, &SonarMapper::AddScanlineToMap, this);
}

void SonarMapper::AddScanlineToMap(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  scanline_count_ ++;

  Eigen::Affine3d transform;
  // The transform order is:
  // 1) rotate the point
  // 2) translate it to the submarine's position
  // 3) Scale it to the map's pixel by meter
  // 4) Transport it to the center of the map (since the Mat center is width/2 an cols/2
  // Eigen take operation from the left to the right
  transform = Eigen::Translation<double, 3>(sonar_map_.rows/2,sonar_map_.cols/2,0)*
              Eigen::Scaling ((double)NB_PIXEL_BY_METER) *
              Eigen::Translation<double, 3>(submarine_position_.GetPosition()) *
              submarine_position_.GetQuaternion();

  float x = 0, y = 0, z = 0, intensity = 0;
  int max_size = (int)(msg->data.size() / msg->point_step);
  for (int i = 0; i < max_size; i++) {
    // Fetch new coordiates
    ExtractNewPoint(msg, i, intensity, x, y, z);
    // transform it in the image coordinate
    Eigen::Vector3d point_cloud_coords (x,y,z);
    Eigen::Vector3d image_coords = transform * point_cloud_coords;

    // Can do this because <at> function returns reference.
    uint8_t &value = sonar_map_.at<uint8_t>((int)image_coords.x(), (int)image_coords.y());
    // Scale the intensity to fit image's normal range.
    // We use 16 bit to prevent overflow if 255+255
    value = (uint8_t)( ( (uint16_t)(intensity * 255.0f) + (uint16_t)value)/2 );
  }

  if( scanline_count_ > MAX_SCANLINE )
  {
    scanline_count_ = 0;
    image_publisher_.Publish(sonar_map_);
  }
}


}