//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xnrm2.h
//
// Code generation for function 'xnrm2'
//

#ifndef XNRM2_H
#define XNRM2_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
double b_xnrm2(int n, const double x[16], int ix0);

float xnrm2(int n, const float x[9], int ix0);

float xnrm2(const float x[3]);

double xnrm2(int n, const double x[3]);

double xnrm2(int n, const double x[9], int ix0);

double xnrm2(const double x[3]);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
// End of code generation (xnrm2.h)
