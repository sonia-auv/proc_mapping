//
// Created by root on 5/26/16.
//

#ifndef PROC_MAPPING_HISTOGRAM_H
#define PROC_MAPPING_HISTOGRAM_H

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

//const int thresh_value_max = 255;
//int thresh_value = 0;

class Histogram : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Histogram>;
  using ConstPtr = std::shared_ptr<const Histogram>;
  using PtrList = std::vector<Histogram::Ptr>;
  using ConstPtrList = std::vector<Histogram::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  Histogram() { };

  virtual ~Histogram() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    int hist_size = 256;
    float range[] = {0, 255};
    const float *hist_range = {range};

    cv::Mat hist;

    cv::calcHist(&input, 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range,
                 true, false);

    int hist_w = 1024;
    int hist_h = 400;
    int bin_w = cvRound(static_cast<double>(hist_w/hist_size));

    cv::Mat hist_image(hist_h, hist_w, CV_8UC1, cv::Scalar(0));

    for(int i = 1; i < hist_size; i++) {
      cv::line(hist_image, cv::Point(bin_w * (i - 1),
           hist_h - cvRound(hist.at<float>(i - 1))), cv::Point(bin_w * (i),
           hist_h - cvRound(hist.at<float>(i))), cv::Scalar(255), 2, 8, 0);
    }

    float thresh = (static_cast<float>(thresh_value) / static_cast<float>(thresh_value_max));

    cv::line(hist_image, cv::Point(hist_w * thresh, 0),
             cv::Point(hist_w * thresh, hist_h),
             cv::Scalar(255));

      cv::imshow("Histogram", hist_image);
      cv::waitKey(1);
  }
};

}  // namespace proc_mapping

#endif //PROC_MAPPING_HISTOGRAM_H
