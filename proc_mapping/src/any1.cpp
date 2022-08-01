//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: any1.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "any1.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<bool, 2U> &x
//                ::coder::array<bool, 1U> &y
// Return Type  : void
//
namespace coder {
void any(const ::coder::array<bool, 2U> &x, ::coder::array<bool, 1U> &y)
{
  int i1;
  int i2;
  int vstride;
  y.set_size(x.size(0));
  vstride = x.size(0);
  for (i2 = 0; i2 < vstride; i2++) {
    y[i2] = false;
  }
  vstride = x.size(0);
  i2 = x.size(0) << 1;
  i1 = 0;
  for (int j{0}; j < vstride; j++) {
    int ix;
    bool exitg1;
    i1++;
    i2++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && ((vstride > 0) && (ix <= i2))) {
      if (x[ix - 1]) {
        y[j] = true;
        exitg1 = true;
      } else {
        ix += vstride;
      }
    }
  }
}

//
// Arguments    : const bool x[22317]
//                bool y[7439]
// Return Type  : void
//
void b_any(const bool x[22317], bool y[7439])
{
  int i1;
  int i2;
  i1 = 0;
  i2 = 14878;
  for (int j{0}; j < 7439; j++) {
    int ix;
    bool exitg1;
    y[j] = false;
    i1++;
    i2++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2)) {
      if (x[ix - 1]) {
        y[j] = true;
        exitg1 = true;
      } else {
        ix += 7439;
      }
    }
  }
}

} // namespace coder

//
// File trailer for any1.cpp
//
// [EOF]
//
