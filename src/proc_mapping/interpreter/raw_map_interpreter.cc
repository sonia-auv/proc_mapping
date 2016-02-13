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
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  pcl::fromROSMsg(*msg_in, cloud);
  pcl::transformPointCloud(cloud, cloud_out, Tm);
}

//------------------------------------------------------------------------------
//
Eigen::Affine3d RawMapInterpreter::GetTransformationMatrix(
    const nav_msgs::Odometry::ConstPtr &odom) ATLAS_NOEXCEPT {
  Eigen::Quaterniond qd;
  qd.x() = odom->pose.pose.orientation.x;
  qd.y() = odom->pose.pose.orientation.y;
  qd.z() = odom->pose.pose.orientation.z;
  qd.w() = odom->pose.pose.orientation.w;

  current_odom_ = Eigen::Translation3d(odom->pose.pose.position.x,
                                       odom->pose.pose.position.y,
                                       odom->pose.pose.position.z) *
                  qd;

  Eigen::Affine3d Tm;
  if (nb_added_clouds_ == 0) {
    Tm.setIdentity();
  } else {
    Tm = last_odom_.inverse() * current_odom_;
  }

  return Tm;
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
