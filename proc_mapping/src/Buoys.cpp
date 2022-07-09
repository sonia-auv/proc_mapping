//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Buoys.cpp
//
// Code generation for function 'Buoys'
//

// Include files
#include "Buoys.h"
#include "Kdtree.h"
#include "pointCloud.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
coder::vision::internal::codegen::Kdtree binary_expand_op(
    coder::pointCloud *in1, const coder::array<unsigned int, 1U> &in2,
    const coder::array<int, 2U> &in3,
    coder::vision::internal::codegen::Kdtree in4, coder::pointCloud *in5)
{
  coder::array<bool, 2U> b_in2;
  int aux_1_1;
  int i;
  int in2_idx_0;
  int loop_ub;
  int stride_1_1;
  in2_idx_0 = in2.size(0);
  if (in3.size(1) == 1) {
    i = 1;
  } else {
    i = in3.size(1);
  }
  b_in2.set_size(in2_idx_0, i);
  stride_1_1 = (in3.size(1) != 1);
  aux_1_1 = 0;
  if (in3.size(1) == 1) {
    loop_ub = 1;
  } else {
    loop_ub = in3.size(1);
  }
  for (i = 0; i < loop_ub; i++) {
    for (int i1{0}; i1 < in2_idx_0; i1++) {
      b_in2[i1 + b_in2.size(0) * i] =
          (static_cast<double>(in2[i1]) == in3[aux_1_1]);
    }
    aux_1_1 += stride_1_1;
  }
  in1->b_select(b_in2, &in4, in5);
  return in4;
}

// End of code generation (Buoys.cpp)
