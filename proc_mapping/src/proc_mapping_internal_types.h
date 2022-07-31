//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: proc_mapping_internal_types.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef PROC_MAPPING_INTERNAL_TYPES_H
#define PROC_MAPPING_INTERNAL_TYPES_H

// Include Files
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
  double boxSize;
};

struct c_struct_T {
  b_struct_T general;
  double freqThreshold;
};

struct d_struct_T {
  double nBin;
  double nBinsToFilterOut;
};

struct e_struct_T {
  b_struct_T general;
  d_struct_T histogram_filter;
};

struct f_struct_T {
  e_struct_T sonar;
  c_struct_T hydro;
  b_struct_T general;
};

struct g_struct_T {
  double clusterDist;
};

struct h_struct_T {
  double clusterDist;
  double planeTol;
  double icpInlierRatio;
  double zNormalThres;
  double inPlaneThres;
  double minArea;
  double maxArea;
  double gap;
};

struct i_struct_T {
  double clusterDist;
  double topAreaMin;
  double topAreaMax;
  double poseDepth;
  double maxBetweenDist;
  double maxBetweenAngle;
  double squarenessRatio;
};

struct j_struct_T {
  h_struct_T buoys;
  i_struct_T tables;
  g_struct_T hydro;
};

struct k_struct_T {
  double x;
  double y;
  double z;
};

struct l_struct_T {
  double pingerDepth;
  k_struct_T translation;
};

struct m_struct_T {
  k_struct_T translation;
};

struct n_struct_T {
  l_struct_T hydro;
  m_struct_T sonar;
};

struct o_struct_T {
  struct_T preprocessing;
  f_struct_T filter;
  j_struct_T segmentation;
  n_struct_T parameters;
};

struct p_struct_T {
  int xstart;
  int xend;
};

struct cell_wrap_17 {
  coder::array<char, 2U> f1;
};

struct cell_wrap_46 {
  coder::array<double, 2U> f1;
};

struct q_struct_T {
  coder::array<float, 1U> D;
  coder::array<unsigned int, 1U> b_I;
};

struct r_struct_T {
  coder::array<double, 1U> D;
  coder::array<unsigned int, 1U> b_I;
};

#endif
//
// File trailer for proc_mapping_internal_types.h
//
// [EOF]
//
