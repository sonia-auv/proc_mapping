//
// Created by jeremie on 7/27/16.
//

#include "AsyncImagePublisher.h"


AsyncImagePublisher::AsyncImagePublisher( const std::string &topic_name)
    : topic_name_(topic_name),
      image_publisher_(topic_name)
{
  image_publisher_.Start();
  Start();
}

AsyncImagePublisher::~AsyncImagePublisher()
{
  Stop();
  image_publisher_.Stop();
}

void AsyncImagePublisher::Publish(const cv::Mat &image)
{
  image_queue_mutex_.lock();
  images_to_publish_.push(image);
  image_queue_mutex_.unlock();
}

void AsyncImagePublisher::Run()
{
  while( !MustStop() )
  {
    image_queue_mutex_.lock();
    size_t size = images_to_publish_.size();
    image_queue_mutex_.unlock();
    // No image, wait a bit.
    if( size == 0 )
    {
      usleep(1000);
    }else if( size > 10 )
    {
      ROS_ERROR("Too much image to publish, clearing the buffer on %s",
                topic_name_.c_str());
      image_queue_mutex_.lock();
      while(!images_to_publish_.empty())
      {
        images_to_publish_.pop();
      }
      image_queue_mutex_.unlock();

    }else
    {
      image_queue_mutex_.lock();
      cv::Mat tmp_image(images_to_publish_.front());
      images_to_publish_.pop();
      image_queue_mutex_.unlock();
      image_publisher_.Write(tmp_image);
    }
  }
};
