/**
 * \file	async_image_publisher.cc
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

#include "async_image_publisher.h"

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
AsyncImagePublisher::AsyncImagePublisher(const std::string &topic_name)
    : topic_name_(topic_name), image_publisher_(topic_name) {
  image_publisher_.Start();
  Start();
}

//------------------------------------------------------------------------------
//
AsyncImagePublisher::~AsyncImagePublisher() {
  Stop();
  image_publisher_.Stop();
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void AsyncImagePublisher::Publish(const cv::Mat &image) {
  image_queue_mutex_.lock();
  images_to_publish_.push(image);
  image_queue_mutex_.unlock();
}

//------------------------------------------------------------------------------
//
void AsyncImagePublisher::Run() {
  while (!MustStop()) {
    image_queue_mutex_.lock();
    size_t size = images_to_publish_.size();
    image_queue_mutex_.unlock();
    // No image, wait a bit.
    if (size == 0) {
      usleep(1000);
    } else if (size > 10) {
      ROS_ERROR("Too much image to publish, clearing the buffer on %s",
                topic_name_.c_str());
      image_queue_mutex_.lock();
      while (!images_to_publish_.empty()) {
        images_to_publish_.pop();
      }
      image_queue_mutex_.unlock();

    } else {
      image_queue_mutex_.lock();
      cv::Mat tmp_image(images_to_publish_.front());
      images_to_publish_.pop();
      image_queue_mutex_.unlock();
      image_publisher_.Write(tmp_image);
    }
  }
}
