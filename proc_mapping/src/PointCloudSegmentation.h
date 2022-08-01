//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PointCloudSegmentation.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef POINTCLOUDSEGMENTATION_H
#define POINTCLOUDSEGMENTATION_H

// Include Files
#include "rtwtypes.h"
#include "PCANormalCore_api.hpp"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

}

// Type Definitions
class PointCloudSegmentation {
public:
  static void objectAllignBoudingBox(const double q[4],
                                     const coder::pointCloud *pc,
                                     double box[3]);
};

#endif
//
// File trailer for PointCloudSegmentation.h
//
// [EOF]
//
