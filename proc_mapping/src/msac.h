//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: msac.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef MSAC_H
#define MSAC_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace vision {
namespace internal {
namespace ransac {
void msac(const ::coder::array<double, 2U> &allPoints,
          double params_maxDistance,
          const ::coder::array<double, 2U> &varargin_1, bool *isFound,
          ::coder::array<double, 2U> &bestModelParams);

}
} // namespace internal
} // namespace vision
} // namespace coder

#endif
//
// File trailer for msac.h
//
// [EOF]
//
