//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcdenoise.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef PCDENOISE_H
#define PCDENOISE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class d_pointCloud;

namespace vision {
namespace internal {
namespace codegen {
class Kdtree;

}
} // namespace internal
} // namespace vision
class pointCloud;

} // namespace coder

// Function Declarations
namespace coder {
pointCloud *pcdenoise(d_pointCloud *ptCloudIn,
                      vision::internal::codegen::Kdtree *iobj_0,
                      pointCloud *iobj_1);

}

#endif
//
// File trailer for pcdenoise.h
//
// [EOF]
//
