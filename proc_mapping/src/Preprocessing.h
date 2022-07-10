//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Preprocessing.h
//
// Code generation for function 'Preprocessing'
//

#ifndef PREPROCESSING_H
#define PREPROCESSING_H

// Include files
#include "proc_mapping_internal_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class PointCloudBundler;

// Type Definitions
class Preprocessing {
public:
  struct_T param;
};

// Function Declarations
void b_binary_expand_op(coder::array<bool, 1U> &in1,
                        const coder::array<double, 2U> &in2,
                        const PointCloudBundler *in3,
                        const coder::array<double, 1U> &in4,
                        const coder::array<double, 1U> &in5);

void binary_expand_op(coder::array<double, 1U> &in1,
                      const coder::array<double, 1U> &in2,
                      const coder::array<double, 1U> &in3);

#endif
// End of code generation (Preprocessing.h)
