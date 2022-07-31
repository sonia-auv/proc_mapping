//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<float, 2U> &A
//                const float B[9]
//                ::coder::array<float, 2U> &C
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void mtimes(const ::coder::array<float, 2U> &A, const float B[9],
            ::coder::array<float, 2U> &C)
{
  int inner;
  int mc;
  mc = A.size(0);
  inner = A.size(1);
  C.set_size(A.size(0), 3);
  for (int j{0}; j < 3; j++) {
    int boffset;
    int coffset;
    coffset = j * mc;
    boffset = j * 3;
    for (int i{0}; i < mc; i++) {
      C[coffset + i] = 0.0F;
    }
    for (int k{0}; k < inner; k++) {
      float bkj;
      int aoffset;
      aoffset = k * A.size(0);
      bkj = B[boffset + k];
      for (int i{0}; i < mc; i++) {
        int b_i;
        b_i = coffset + i;
        C[b_i] = C[b_i] + A[aoffset + i] * bkj;
      }
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for mtimes.cpp
//
// [EOF]
//
