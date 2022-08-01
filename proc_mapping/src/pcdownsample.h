//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcdownsample.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef PCDOWNSAMPLE_H
#define PCDOWNSAMPLE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

class d_pointCloud;

} // namespace coder

// Function Declarations
namespace coder {
d_pointCloud *pcdownsample(const pointCloud *ptCloudIn, double varargin_2,
                           d_pointCloud *iobj_0);

}

#endif
//
// File trailer for pcdownsample.h
//
// [EOF]
//
