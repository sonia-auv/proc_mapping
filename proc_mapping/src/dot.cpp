//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: dot.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "dot.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const double a[3]
//                const double b[3]
// Return Type  : double
//
namespace coder {
double dot(const double a[3], const double b[3])
{
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

//
// Arguments    : const ::coder::array<double, 2U> &a
//                const double b[3]
//                ::coder::array<double, 2U> &c
// Return Type  : void
//
void dot(const ::coder::array<double, 2U> &a, const double b[3],
         ::coder::array<double, 2U> &c)
{
  if ((a.size(0) == 1) || (a.size(1) == 1)) {
    double b_c;
    int lowerDim;
    lowerDim = a.size(0) * a.size(1);
    b_c = 0.0;
    if (lowerDim >= 1) {
      for (int k{0}; k < lowerDim; k++) {
        b_c += a[k] * b[k];
      }
    }
    c.set_size(1, 1);
    c[0] = b_c;
  } else if ((a.size(0) == 0) && (a.size(1) == 0)) {
    c.set_size(1, 1);
    c[0] = 0.0;
  } else {
    int dim;
    dim = 1;
    if (a.size(0) != 1) {
      dim = 0;
    }
    if (a.size(dim) == 1) {
      int lowerDim;
      c.set_size(a.size(0), a.size(1));
      lowerDim = a.size(0) * a.size(1);
      for (int j{0}; j < lowerDim; j++) {
        c[j] = a[j] * b[j];
      }
    } else {
      unsigned int sz[2];
      int ic;
      int lowerDim;
      int npages;
      int vlen;
      int vspread;
      int vstride;
      sz[0] = static_cast<unsigned int>(a.size(0));
      sz[1] = static_cast<unsigned int>(a.size(1));
      sz[dim] = 1U;
      c.set_size(static_cast<int>(sz[0]), static_cast<int>(sz[1]));
      vlen = a.size(dim);
      vstride = 1;
      for (int k{0}; k < dim; k++) {
        vstride *= a.size(0);
      }
      vspread = (a.size(dim) - 1) * vstride;
      npages = 1;
      lowerDim = dim + 2;
      for (int k{lowerDim}; k < 3; k++) {
        npages *= a.size(1);
      }
      lowerDim = 0;
      ic = -1;
      for (int i{0}; i < npages; i++) {
        int i1;
        i1 = lowerDim - 1;
        lowerDim += vspread;
        for (int j{0}; j < vstride; j++) {
          double b_c;
          i1++;
          lowerDim++;
          b_c = 0.0;
          if (a.size(dim) >= 1) {
            int ix;
            int iy;
            ix = i1;
            iy = i1;
            for (int k{0}; k < vlen; k++) {
              b_c += a[ix] * b[iy];
              ix += vstride;
              iy += vstride;
            }
          }
          c[(ic + j) + 1] = b_c;
        }
        ic += vstride;
      }
    }
  }
}

} // namespace coder

//
// File trailer for dot.cpp
//
// [EOF]
//
