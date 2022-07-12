//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping_internal_types.h
//
// Code generation for function 'proc_mapping'
//

#ifndef PROC_MAPPING_INTERNAL_TYPES_H
#define PROC_MAPPING_INTERNAL_TYPES_H

// Include files
#include "proc_mapping_types.h"
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct struct_T {
  double minIntensity;
  double maxIntensity;
  double minRange;
  double maxRange;
};

struct b_struct_T {
  double clusterDist;
  double planeTol;
  double icpInlierRatio;
  double zNormalThres;
  double inPlaneThres;
  double minArea;
  double maxArea;
};

struct c_struct_T {
  b_struct_T buoys;
};

struct d_struct_T {
  double boxSize;
};

struct e_struct_T {
  d_struct_T general;
};

struct f_struct_T {
  struct_T preprocessing;
  e_struct_T filter;
  c_struct_T segmentation;
};

struct g_struct_T {
  int xstart;
  int xend;
};

struct cell_wrap_17 {
  coder::array<char, 2U> f1;
};

struct cell_wrap_41 {
  coder::array<double, 2U> f1;
};

struct h_struct_T {
  coder::array<double, 1U> D;
  coder::array<unsigned int, 1U> b_I;
};

struct i_struct_T {
  coder::array<float, 1U> D;
  coder::array<unsigned int, 1U> b_I;
};

#endif
// End of code generation (proc_mapping_internal_types.h)
