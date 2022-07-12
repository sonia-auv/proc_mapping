//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sortedInsertion.h
//
// Code generation for function 'sortedInsertion'
//

#ifndef SORTEDINSERTION_H
#define SORTEDINSERTION_H

// Include files
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
// End of code generation (sortedInsertion.h)
