//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: var.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "var.h"
#include "blockedSummation.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 1U> &x
// Return Type  : double
//
namespace coder {
double var(const ::coder::array<double, 1U> &x)
{
  double y;
  int n;
  n = x.size(0);
  if (x.size(0) == 0) {
    y = rtNaN;
  } else if (x.size(0) == 1) {
    if ((!std::isinf(x[0])) && (!std::isnan(x[0]))) {
      y = 0.0;
    } else {
      y = rtNaN;
    }
  } else {
    double xbar;
    xbar = blockedSummation(x, x.size(0)) / static_cast<double>(x.size(0));
    y = 0.0;
    for (int k{0}; k < n; k++) {
      double t;
      t = x[k] - xbar;
      y += t * t;
    }
    y /= static_cast<double>(x.size(0)) - 1.0;
  }
  return y;
}

} // namespace coder

//
// File trailer for var.cpp
//
// [EOF]
//
