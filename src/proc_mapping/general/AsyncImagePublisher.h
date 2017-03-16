/**
 * \file	AsyncImagePublisher.h
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	27/07/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
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

#ifndef PROC_MAPPING_ASYNC_IMAGE_PUBLISHER_H_
#define PROC_MAPPING_ASYNC_IMAGE_PUBLISHER_H_

#include <queue>

#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/ros/image_publisher.h>
#include <ros/ros.h>
#include <mutex>
#include <opencv2/opencv.hpp>

class AsyncImagePublisher : public atlas::Runnable {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<AsyncImagePublisher>;
  using ConstPtr = std::shared_ptr<const AsyncImagePublisher>;
  using PtrList = std::vector<AsyncImagePublisher::Ptr>;
  using ConstPtrList = std::vector<AsyncImagePublisher::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit AsyncImagePublisher(const std::string &topic_name);
  ~AsyncImagePublisher();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Publish(const cv::Mat &image);

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void Run() override;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string topic_name_;
  atlas::ImagePublisher image_publisher_;

  std::queue<cv::Mat> images_to_publish_;
  std::mutex image_queue_mutex_;
};

#endif  // PROC_MAPPING_ASYNC_IMAGE_PUBLISHER_H_
