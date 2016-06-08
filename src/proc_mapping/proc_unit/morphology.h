/**
 * \file	morphology.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	06/06/2016
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

#ifndef PROC_MAPPING_MORPHOLOGY_H
#define PROC_MAPPING_MORPHOLOGY_H

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class Morphology : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Morphology>;
  using ConstPtr = std::shared_ptr<const Morphology>;
  using PtrList = std::vector<Morphology::Ptr>;
  using ConstPtrList = std::vector<Morphology::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Morphology(bool debug = false) : debug(debug) {}

  virtual ~Morphology() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    cv::Mat element = cv::getStructuringElement(
        0, cv::Size(2 * 2 + 1, 2 * 2 + 1), cv::Point(2, 2));

    cv::morphologyEx(input, input, cv::MORPH_CLOSE, element);

    if (debug) {
      cv::imshow("Morphology", input);
      cv::waitKey(1);
    }
  }

 private:
  bool debug;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_MORPHOLOGY_H
