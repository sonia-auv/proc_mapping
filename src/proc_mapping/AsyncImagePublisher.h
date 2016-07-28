//
// Created by jeremie on 7/27/16.
//

#ifndef PROC_MAPPING_ASYNCIMAGEPUBLISHER_HPP
#define PROC_MAPPING_ASYNCIMAGEPUBLISHER_HPP

#include <queue>

#include <lib_atlas/pattern/runnable.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <lib_atlas/ros/image_publisher.h>
#include <mutex>

class AsyncImagePublisher : public atlas::Runnable {
 public:
  AsyncImagePublisher(const std::string &topic_name);
  ~AsyncImagePublisher();

  void Publish(const cv::Mat &image);
 private:

  void Run() override ;

  std::string topic_name_;
  atlas::ImagePublisher image_publisher_;

  std::queue<cv::Mat> images_to_publish_;
  std::mutex image_queue_mutex_;

};


#endif //PROC_MAPPING_ASYNCIMAGEPUBLISHER_HPP
