//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sortedInsertion.cpp
//
// Code generation for function 'sortedInsertion'
//

// Include files
#include "sortedInsertion.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
namespace internal {
void sortedInsertion(double x, int ix, ::coder::array<double, 1U> &b, int *nb,
                     int blen, ::coder::array<int, 1U> &idx)
{
  if (blen != 0) {
    if (*nb == 0) {
      *nb = 1;
      idx[0] = ix;
      b[0] = x;
    } else if ((x >= b[0]) || std::isnan(x)) {
      if ((*nb > 1) && (!(x >= b[*nb - 1])) && (!std::isnan(x))) {
        int ja;
        int jb;
        int jc;
        ja = 1;
        jb = *nb;
        while (ja < jb) {
          jc = ja + ((jb - ja) >> 1);
          if (jc == ja) {
            ja = jb;
          } else if ((x >= b[jc - 1]) || std::isnan(x)) {
            ja = jc;
          } else {
            jb = jc;
          }
        }
        if (*nb < blen) {
          (*nb)++;
        }
        jb = ja + 1;
        for (jc = *nb; jc >= jb; jc--) {
          b[jc - 1] = b[jc - 2];
          idx[jc - 1] = idx[jc - 2];
        }
        b[ja - 1] = x;
        idx[ja - 1] = ix;
      } else if (*nb < blen) {
        (*nb)++;
        b[*nb - 1] = x;
        idx[*nb - 1] = ix;
      }
    } else {
      if (*nb < blen) {
        (*nb)++;
      }
      for (int jc{*nb}; jc >= 2; jc--) {
        idx[jc - 1] = idx[jc - 2];
        b[jc - 1] = b[jc - 2];
      }
      b[0] = x;
      idx[0] = ix;
    }
  }
}

} // namespace internal
} // namespace coder

// End of code generation (sortedInsertion.cpp)
