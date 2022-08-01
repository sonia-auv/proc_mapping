//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeRigidTransform.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef COMPUTERIGIDTRANSFORM_H
#define COMPUTERIGIDTRANSFORM_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace vision {
namespace internal {
namespace calibration {
void computeRigidTransform(const ::coder::array<float, 2U> &p,
                           const ::coder::array<double, 2U> &q, float R[9],
                           float t[3]);

}
} // namespace internal
} // namespace vision
} // namespace coder

#endif
//
// File trailer for computeRigidTransform.h
//
// [EOF]
//
