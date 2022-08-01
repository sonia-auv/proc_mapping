//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef SVD1_H
#define SVD1_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_svd(const float A[9], float U[3]);

void b_svd(const float A[9], float U[9], float s[3], float V[9]);

void b_svd(const double A[9], double U[9], double s[3], double V[9]);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for svd1.h
//
// [EOF]
//
