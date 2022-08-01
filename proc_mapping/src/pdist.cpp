//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pdist.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "pdist.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const double Xin[6]
// Return Type  : double
//
namespace coder {
double pdist(const double Xin[6])
{
  double X[6];
  double Y;
  int jj;
  bool logIndX[2];
  bool exitg1;
  bool nanflag;
  for (jj = 0; jj < 2; jj++) {
    X[3 * jj] = Xin[jj];
    X[3 * jj + 1] = Xin[jj + 2];
    X[3 * jj + 2] = Xin[jj + 4];
    logIndX[jj] = true;
  }
  nanflag = false;
  jj = 0;
  exitg1 = false;
  while ((!exitg1) && (jj < 3)) {
    if (std::isnan(X[jj])) {
      nanflag = true;
      exitg1 = true;
    } else {
      jj++;
    }
  }
  if (nanflag) {
    logIndX[0] = false;
  }
  nanflag = false;
  jj = 0;
  exitg1 = false;
  while ((!exitg1) && (jj < 3)) {
    if (std::isnan(X[jj + 3])) {
      nanflag = true;
      exitg1 = true;
    } else {
      jj++;
    }
  }
  if (nanflag) {
    logIndX[1] = false;
  }
  Y = rtNaN;
  if (logIndX[0] && logIndX[1]) {
    double b_tempSum_tmp;
    double c_tempSum_tmp;
    double tempSum_tmp;
    tempSum_tmp = X[0] - X[3];
    b_tempSum_tmp = X[1] - X[4];
    c_tempSum_tmp = X[2] - X[5];
    Y = std::sqrt((tempSum_tmp * tempSum_tmp + b_tempSum_tmp * b_tempSum_tmp) +
                  c_tempSum_tmp * c_tempSum_tmp);
  }
  return Y;
}

} // namespace coder

//
// File trailer for pdist.cpp
//
// [EOF]
//
