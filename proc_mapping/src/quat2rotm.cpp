//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quat2rotm.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "quat2rotm.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const double q[4]
//                double R[9]
// Return Type  : void
//
namespace coder {
void quat2rotm(const double q[4], double R[9])
{
  double tempR[9];
  double b;
  double b_tempR_tmp;
  double c_tempR_tmp;
  double d_tempR_tmp;
  double e_tempR_tmp;
  double f_tempR_tmp;
  double normRowMatrix_idx_0;
  double normRowMatrix_idx_1;
  double normRowMatrix_idx_2;
  double tempR_tmp;
  b = 1.0 /
      std::sqrt(((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3]);
  normRowMatrix_idx_0 = q[0] * b;
  normRowMatrix_idx_1 = q[1] * b;
  normRowMatrix_idx_2 = q[2] * b;
  b *= q[3];
  tempR_tmp = b * b;
  b_tempR_tmp = normRowMatrix_idx_2 * normRowMatrix_idx_2;
  tempR[0] = 1.0 - 2.0 * (b_tempR_tmp + tempR_tmp);
  c_tempR_tmp = normRowMatrix_idx_1 * normRowMatrix_idx_2;
  d_tempR_tmp = normRowMatrix_idx_0 * b;
  tempR[1] = 2.0 * (c_tempR_tmp - d_tempR_tmp);
  e_tempR_tmp = normRowMatrix_idx_1 * b;
  f_tempR_tmp = normRowMatrix_idx_0 * normRowMatrix_idx_2;
  tempR[2] = 2.0 * (e_tempR_tmp + f_tempR_tmp);
  tempR[3] = 2.0 * (c_tempR_tmp + d_tempR_tmp);
  c_tempR_tmp = normRowMatrix_idx_1 * normRowMatrix_idx_1;
  tempR[4] = 1.0 - 2.0 * (c_tempR_tmp + tempR_tmp);
  tempR_tmp = normRowMatrix_idx_2 * b;
  d_tempR_tmp = normRowMatrix_idx_0 * normRowMatrix_idx_1;
  tempR[5] = 2.0 * (tempR_tmp - d_tempR_tmp);
  tempR[6] = 2.0 * (e_tempR_tmp - f_tempR_tmp);
  tempR[7] = 2.0 * (tempR_tmp + d_tempR_tmp);
  tempR[8] = 1.0 - 2.0 * (c_tempR_tmp + b_tempR_tmp);
  std::copy(&tempR[0], &tempR[9], &R[0]);
  for (int k{0}; k < 3; k++) {
    R[k] = tempR[3 * k];
    R[k + 3] = tempR[3 * k + 1];
    R[k + 6] = tempR[3 * k + 2];
  }
}

} // namespace coder

//
// File trailer for quat2rotm.cpp
//
// [EOF]
//
