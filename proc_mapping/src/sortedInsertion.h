//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sortedInsertion.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef SORTEDINSERTION_H
#define SORTEDINSERTION_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void sortedInsertion(double x, int ix, ::coder::array<double, 1U> &b, int *nb,
                     int blen, ::coder::array<int, 1U> &idx);

}
} // namespace coder

#endif
//
// File trailer for sortedInsertion.h
//
// [EOF]
//
