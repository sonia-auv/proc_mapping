//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: subspace.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "subspace.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const double A[2]
//                double U[2]
//                int *r
// Return Type  : void
//
namespace coder {
void modified_orth(const double A[2], double U[2], int *r)
{
  double scale;
  bool p;
  p = true;
  if (std::isinf(A[0]) || std::isnan(A[0]) ||
      (std::isinf(A[1]) || std::isnan(A[1]))) {
    p = false;
  }
  if (p) {
    double A_idx_0;
    double A_idx_1;
    double absxk;
    double nrm;
    double t;
    scale = 3.3121686421112381E-170;
    A_idx_0 = A[0];
    absxk = std::abs(A[0]);
    if (absxk > 3.3121686421112381E-170) {
      nrm = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      nrm = t * t;
    }
    A_idx_1 = A[1];
    absxk = std::abs(A[1]);
    if (absxk > scale) {
      t = scale / absxk;
      nrm = nrm * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      nrm += t * t;
    }
    nrm = scale * std::sqrt(nrm);
    if (nrm > 0.0) {
      if (A[0] < 0.0) {
        scale = -nrm;
      } else {
        scale = nrm;
      }
      if (std::abs(scale) >= 1.0020841800044864E-292) {
        absxk = 1.0 / scale;
        A_idx_0 = absxk * A[0];
        A_idx_1 = absxk * A[1];
      } else {
        A_idx_0 = A[0] / scale;
        A_idx_1 = A[1] / scale;
      }
      A_idx_0++;
      scale = -scale;
    } else {
      scale = 0.0;
    }
    if (scale != 0.0) {
      t = std::abs(scale);
      absxk = scale / t;
      scale = t;
      U[0] = absxk * (-A_idx_0 + 1.0);
      U[1] = absxk * -A_idx_1;
    } else {
      U[1] = 0.0;
      U[0] = 1.0;
    }
  } else {
    U[0] = rtNaN;
    U[1] = rtNaN;
    scale = rtNaN;
  }
  *r = 0;
  if (scale > 2.0 * scale * 2.2204460492503131E-16) {
    *r = 1;
  }
}

} // namespace coder

//
// File trailer for subspace.cpp
//
// [EOF]
//
