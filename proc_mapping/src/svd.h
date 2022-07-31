//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef SVD_H
#define SVD_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void svd(const double A[9], double U[9], double S[9], double V[9]);

void svd(const double A[9], double U[3]);

} // namespace coder

#endif
//
// File trailer for svd.h
//
// [EOF]
//
