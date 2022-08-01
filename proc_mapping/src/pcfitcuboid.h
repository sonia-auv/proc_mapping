//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcfitcuboid.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef PCFITCUBOID_H
#define PCFITCUBOID_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

class cuboidModel;

} // namespace coder

// Function Declarations
namespace coder {
cuboidModel *pcfitcuboid(const pointCloud *ptCloudIn,
                         const ::coder::array<double, 1U> &varargin_1,
                         cuboidModel *iobj_0);

}

#endif
//
// File trailer for pcfitcuboid.h
//
// [EOF]
//
