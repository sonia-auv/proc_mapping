//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quatinv.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "quatinv.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : double q[4]
// Return Type  : void
//
namespace coder {
void quatinv(double q[4])
{
  double qnrm;
  qnrm = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
  q[0] /= qnrm;
  q[1] = -q[1] / qnrm;
  q[2] = -q[2] / qnrm;
  q[3] = -q[3] / qnrm;
}

} // namespace coder

//
// File trailer for quatinv.cpp
//
// [EOF]
//
