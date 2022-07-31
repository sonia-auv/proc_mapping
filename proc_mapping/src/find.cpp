//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: find.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "find.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<bool, 1U> &x
//                ::coder::array<int, 1U> &i
// Return Type  : void
//
namespace coder {
void b_eml_find(const ::coder::array<bool, 1U> &x, ::coder::array<int, 1U> &i)
{
  int idx;
  int ii;
  int nx;
  bool exitg1;
  nx = x.size(0);
  idx = 0;
  i.set_size(x.size(0));
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nx - 1)) {
    if (x[ii]) {
      idx++;
      i[idx - 1] = ii + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (x.size(0) == 1) {
    if (idx == 0) {
      i.set_size(0);
    }
  } else {
    if (idx < 1) {
      idx = 0;
    }
    i.set_size(idx);
  }
}

//
// Arguments    : const ::coder::array<bool, 2U> &x
//                ::coder::array<int, 2U> &i
// Return Type  : void
//
void eml_find(const ::coder::array<bool, 2U> &x, ::coder::array<int, 2U> &i)
{
  int idx;
  int ii;
  int nx;
  bool exitg1;
  nx = x.size(1);
  idx = 0;
  i.set_size(1, x.size(1));
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nx - 1)) {
    if (x[ii]) {
      idx++;
      i[idx - 1] = ii + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (x.size(1) == 1) {
    if (idx == 0) {
      i.set_size(1, 0);
    }
  } else {
    if (idx < 1) {
      idx = 0;
    }
    i.set_size(i.size(0), idx);
  }
}

} // namespace coder

//
// File trailer for find.cpp
//
// [EOF]
//
