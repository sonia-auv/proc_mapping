/**
 * \file	raw_map_interpreter.cc
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

#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include "proc_mapping/interpreter/raw_map_interpreter.h"

namespace proc_mapping {

//==============================================================================
// T Y P E D E F   A N D   E N U M

const uint16_t RawMapInterpreter::FRAME_SYNC = 20;

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RawMapInterpreter::RawMapInterpreter(const ros::NodeHandlePtr &nh)
    ATLAS_NOEXCEPT : DataInterpreter<cv::Mat>(nh),
                     nh_(nh),
                     points2_sub_(),
                     odom_sub_() {
  std::string points_topic;
  std::string odometry_topic;

  nh->param<std::string>("topics/points_topic", points_topic,
                         "/provider_sonar/point_cloud2");
  nh->param<std::string>("topics/odometry", odometry_topic,
                         "/proc_navigation/odom");

  int w, h, r;
  nh->param<int>("map/width", w, 100);
  nh->param<int>("map/height", h, 100);
  nh->param<int>("map/res", r, 100);
  map_.width = static_cast<uint32_t >(w);
  map_.width = static_cast<uint32_t >(h);
  map_.width = static_cast<uint32_t >(r);

  points2_sub_ =
      std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
          *(nh_.get()), points_topic, 1);
  odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(
      *(nh_.get()), odometry_topic, 10);
  sync_po_ =
      std::make_shared<message_filters::Synchronizer<PointsOdomSyncPolicies>>(
          PointsOdomSyncPolicies(FRAME_SYNC), *points2_sub_, *odom_sub_);
  sync_po_->registerCallback(
      boost::bind(&RawMapInterpreter::PointCloud2OdomCallback, this, _1, _2));
}

//------------------------------------------------------------------------------
//
RawMapInterpreter::~RawMapInterpreter() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void RawMapInterpreter::PointCloud2OdomCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg_in,
    const nav_msgs::Odometry::ConstPtr &odo_in) ATLAS_NOEXCEPT {
  auto Tm = GetTransformationMatrix(odo_in);
  float x, y, z, intensity;
  auto cloud = *msg_in;
  for (uint64_t i = scanline_threshold_;
       i < cloud.data.size() && i + 3 * sizeof(float) < cloud.data.size();
       i += cloud.point_step) {
    // --
    // Parsing msg data
    memcpy(&x, &cloud.data[i], sizeof(float));
    memcpy(&y, &cloud.data[i + sizeof(float)], sizeof(float));
    memcpy(&z, &cloud.data[i + 2 * sizeof(float)], sizeof(float));
    memcpy(&intensity, &cloud.data[i + 3 * sizeof(float)], sizeof(float));

    Eigen::Vector2d p(x, y);
    auto t = (p * Tm).matrix();

    auto u = ConvertToPixel(p(0));
    auto v = ConvertToPixel(p(1));

    number_of_hits_[u + v * map_.width]++;

    map_.mat.at<uchar>(u, v) =
        (static_cast<uint8_t>(255.0 * intensity) + map_.mat.at<uchar>(u, v)) /
        2;
  }
}

//------------------------------------------------------------------------------
//
Eigen::Matrix3d RawMapInterpreter::GetTransformationMatrix(
    const nav_msgs::Odometry::ConstPtr &odom) ATLAS_NOEXCEPT {
  // We are going to build the transformation matrix with only the yaw. This
  // is due to sonar behavior that is unable to extrapole the Z position of
  // the ray that are received. In this case, we will have to do just like
  // there was no z dimention.
  tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                   odom->pose.pose.orientation.z,
                   odom->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double yaw, pitch, roll;
  m.getRPY(roll, pitch, yaw);

  yaw = M_PI - yaw;
  Eigen::Rotation2D<double> R(yaw);
  Eigen::Vector2d T(odom->pose.pose.position.x,
                               odom->pose.pose.position.y);
  return (R * T).matrix();
}

//------------------------------------------------------------------------------
//
void RawMapInterpreter::SetMapParameters(const uint32_t &map_width,
                                         const uint32_t &map_height,
                                         const float &map_resolution) ATLAS_NOEXCEPT {
  // Setting the new size of the map
  map_.height = static_cast<uint32_t>(map_height / map_resolution);
  map_.width = static_cast<uint32_t>(map_width / map_resolution);
  cv::Size size(map_width, map_height);
  cv::resize(map_.mat, map_.mat, size);
  map_.mat.setTo(cv::Scalar(0));
  map_.res = map_resolution;
  number_of_hits_.resize(map_.height * map_.width);

  // Setting the map transformation matrix
  auto sub_initial_x = map_width / 2;
  auto sub_initial_y = map_height / 2;
  map_.map_transform_ = Eigen::Translation<uint32_t, 2>(
      static_cast<uint32_t>(sub_initial_x / map_resolution),
      static_cast<uint32_t>(sub_initial_y / map_resolution));
}

//------------------------------------------------------------------------------
//
int RawMapInterpreter::ConvertToPixel(float value) { return static_cast<int>
  (value/map_.res); }

//------------------------------------------------------------------------------
//
uint8_t RawMapInterpreter::GetInfiniteMean(uint8_t newIntensity,
                                           uint8_t currentIntensity,
                                           int numberOfHits) {
  if (numberOfHits == 0) {
    return newIntensity;
  }
  float factor = static_cast<float>(1.0 / numberOfHits);
  return static_cast<uint8_t>(float(newIntensity) * factor +
                              float((numberOfHits - 1) * currentIntensity) *
                                  factor);
}

//------------------------------------------------------------------------------
//
pcl::PointCloud<pcl::PointXYZ> RawMapInterpreter::MatToPointXYZ(
    const cv::Mat &m) ATLAS_NOEXCEPT {
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
cv::Mat RawMapInterpreter::PointXYZToMat(
    const pcl::PointCloud<pcl::PointXYZ> &point_cloud) ATLAS_NOEXCEPT {
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
