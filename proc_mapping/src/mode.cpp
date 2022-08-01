//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mode.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "mode.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<unsigned int, 1U> &x
// Return Type  : unsigned int
//
namespace coder {
unsigned int arraymode(const ::coder::array<unsigned int, 1U> &x)
{
  array<unsigned int, 1U> v;
  unsigned int M;
  if (x.size(0) == 0) {
    M = 0U;
  } else {
    int ftmp;
    int i;
    int loop_ub;
    unsigned int mtmp;
    v.set_size(x.size(0));
    loop_ub = x.size(0);
    for (i = 0; i < loop_ub; i++) {
      v[i] = x[i];
    }
    internal::sort(v);
    M = v[0];
    loop_ub = 1;
    mtmp = v[0];
    ftmp = 1;
    i = v.size(0);
    for (int k{0}; k <= i - 2; k++) {
      unsigned int u;
      u = v[k + 1];
      if (u == mtmp) {
        ftmp++;
      } else {
        if (ftmp > loop_ub) {
          M = mtmp;
          loop_ub = ftmp;
        }
        mtmp = u;
        ftmp = 1;
      }
    }
    if (ftmp > loop_ub) {
      M = mtmp;
    }
  }
  return M;
}

} // namespace coder

//
// File trailer for mode.cpp
//
// [EOF]
//
