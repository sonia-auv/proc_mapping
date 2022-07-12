//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// RosNode.cpp
//
// Code generation for function 'RosNode'
//

// Include files
#include "RosNode.h"
#include "ParameterTree.h"
#include "rt_nonfinite.h"
#include "startsWith.h"
#include "coder_array.h"
#include "mlroscpp_param.h"
#include <functional>
#include <string.h>

// Function Definitions
void RosNode::getRosParams(double *param_preprocessing_minIntensity,
                           double *param_preprocessing_maxIntensity,
                           double *param_preprocessing_minRange,
                           double *param_preprocessing_maxRange,
                           double *param_filter_general_boxSize,
                           double *param_segmentation_buoys_clusterDist,
                           double *param_segmentation_buoys_planeTol,
                           double *param_segmentation_buoys_icpInlierRatio,
                           double *param_segmentation_buoys_zNormalThres,
                           double *param_segmentation_buoys_inPlaneThres,
                           double *param_segmentation_buoys_minArea,
                           double *param_segmentation_buoys_maxArea)
{
  static const char m_name[49]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                               'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                               't', 'a', 't', 'i', 'o', 'n', '/', 'b', 'u', 'o',
                               'y', 's', '/', 'i', 'c', 'p', '_', 'i', 'n', 'l',
                               'i', 'e', 'r', '_', 'r', 'a', 't', 'i', 'o'};
  static const char o_name[47]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                               'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                               't', 'a', 't', 'i', 'o', 'n', '/', 'b', 'u', 'o',
                               'y', 's', '/', 'z', '_', 'n', 'o', 'r', 'm', 'a',
                               'l', '_', 't', 'h', 'r', 'e', 's'};
  static const char p_name[47]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                               'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                               't', 'a', 't', 'i', 'o', 'n', '/', 'b', 'u', 'o',
                               'y', 's', '/', 'i', 'n', '_', 'p', 'l', 'a', 'n',
                               'e', '_', 't', 'h', 'r', 'e', 's'};
  static const char i_name[45]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p',
                               'p', 'i', 'n', 'g', '/', 's', 'e', 'g', 'm',
                               'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/',
                               'b', 'u', 'o', 'y', 's', '/', 'c', 'l', 'u',
                               's', 't', 'e', 'r', '_', 'd', 'i', 's', 't'};
  static const char k_name[42]{
      '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/',
      's', 'e', 'g', 'm', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/', 'b',
      'u', 'o', 'y', 's', '/', 'p', 'l', 'a', 'n', 'e', '_', 't', 'o', 'l'};
  static const char b_name[41]{
      '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/',
      'p', 'r', 'e', 'p', 'r', 'o', 'c', 'e', 's', 's', 'i', 'n', 'g', '/',
      'm', 'i', 'n', '_', 'i', 'n', 't', 'e', 'n', 's', 'i', 't', 'y'};
  static const char c_name[41]{
      '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/',
      'p', 'r', 'e', 'p', 'r', 'o', 'c', 'e', 's', 's', 'i', 'n', 'g', '/',
      'm', 'a', 'x', '_', 'i', 'n', 't', 'e', 'n', 's', 'i', 't', 'y'};
  static const char q_name[41]{
      '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/',
      's', 'e', 'g', 'm', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/', 'b',
      'u', 'o', 'y', 's', '/', 'm', 'i', 'n', '_', 'a', 'r', 'e', 'a'};
  static const char e_name[37]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                               'i', 'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o',
                               'c', 'e', 's', 's', 'i', 'n', 'g', '/', 'm', 'i',
                               'n', '_', 'r', 'a', 'n', 'g', 'e'};
  static const char f_name[37]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                               'i', 'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o',
                               'c', 'e', 's', 's', 'i', 'n', 'g', '/', 'm', 'a',
                               'x', '_', 'r', 'a', 'n', 'g', 'e'};
  static const char g_name[37]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                               'i', 'n', 'g', '/', 'f', 'i', 'l', 't', 'e', 'r',
                               '/', 'g', 'e', 'n', 'e', 'r', 'a', 'l', '/', 'b',
                               'o', 'x', '_', 's', 'i', 'z', 'e'};
  coder::ros::ParameterTree lobj_1;
  coder::array<char, 2U> in;
  coder::array<char, 2U> parameterName;
  int loop_ub;
  char l_name[49];
  char n_name[47];
  char h_name[45];
  char j_name[42];
  char name[41];
  char d_name[37];
  bool nameExists;
  //  Preprocessing
  //  Filter
  //  General
  //  Segmentation
  //  Buoys
  lobj_1.ParameterHelper = MATLABROSParameter();
  UNUSED_PARAM(lobj_1.ParameterHelper);
  // ROSPARAM Construct an instance of this class
  //    Detailed explanation goes here
  //  Preprocessing
  //         %% get rosparam number
  for (int i{0}; i < 41; i++) {
    name[i] = b_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &name[0]);
  in.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    in[i] = b_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 40);
    parameterName[0] = '~';
    for (int i{0}; i < 39; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 40);
    for (int i{0}; i < 40; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 41; i++) {
      name[i] = b_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &name[0]);
    in.set_size(1, 41);
    for (int i{0}; i < 41; i++) {
      in[i] = b_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 40);
      parameterName[0] = '~';
      for (int i{0}; i < 39; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 40);
      for (int i{0}; i < 40; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_preprocessing_minIntensity = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_preprocessing_minIntensity);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_preprocessing_minIntensity = 0.1;
  }
  //         %% get rosparam number
  for (int i{0}; i < 41; i++) {
    name[i] = c_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &name[0]);
  in.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    in[i] = c_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 40);
    parameterName[0] = '~';
    for (int i{0}; i < 39; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 40);
    for (int i{0}; i < 40; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 41; i++) {
      name[i] = c_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &name[0]);
    in.set_size(1, 41);
    for (int i{0}; i < 41; i++) {
      in[i] = c_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 40);
      parameterName[0] = '~';
      for (int i{0}; i < 39; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 40);
      for (int i{0}; i < 40; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_preprocessing_maxIntensity = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_preprocessing_maxIntensity);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_preprocessing_maxIntensity = 1.0;
  }
  //         %% get rosparam number
  for (int i{0}; i < 37; i++) {
    d_name[i] = e_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &d_name[0]);
  in.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    in[i] = e_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 36);
    parameterName[0] = '~';
    for (int i{0}; i < 35; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 36);
    for (int i{0}; i < 36; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 37; i++) {
      d_name[i] = e_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &d_name[0]);
    in.set_size(1, 37);
    for (int i{0}; i < 37; i++) {
      in[i] = e_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 36);
      parameterName[0] = '~';
      for (int i{0}; i < 35; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 36);
      for (int i{0}; i < 36; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_preprocessing_minRange = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_preprocessing_minRange);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_preprocessing_minRange = 0.1;
  }
  //         %% get rosparam number
  for (int i{0}; i < 37; i++) {
    d_name[i] = f_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &d_name[0]);
  in.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    in[i] = f_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 36);
    parameterName[0] = '~';
    for (int i{0}; i < 35; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 36);
    for (int i{0}; i < 36; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 37; i++) {
      d_name[i] = f_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &d_name[0]);
    in.set_size(1, 37);
    for (int i{0}; i < 37; i++) {
      in[i] = f_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 36);
      parameterName[0] = '~';
      for (int i{0}; i < 35; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 36);
      for (int i{0}; i < 36; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_preprocessing_maxRange = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_preprocessing_maxRange);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_preprocessing_maxRange = 5.0;
  }
  //  Filter
  //  General
  //         %% get rosparam number
  for (int i{0}; i < 37; i++) {
    d_name[i] = g_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &d_name[0]);
  in.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    in[i] = g_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 36);
    parameterName[0] = '~';
    for (int i{0}; i < 35; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 36);
    for (int i{0}; i < 36; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 37; i++) {
      d_name[i] = g_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &d_name[0]);
    in.set_size(1, 37);
    for (int i{0}; i < 37; i++) {
      in[i] = g_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 36);
      parameterName[0] = '~';
      for (int i{0}; i < 35; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 36);
      for (int i{0}; i < 36; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_filter_general_boxSize = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_filter_general_boxSize);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_filter_general_boxSize = 0.05;
  }
  //  Segmentation
  //  Buoys
  //         %% get rosparam number
  for (int i{0}; i < 45; i++) {
    h_name[i] = i_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &h_name[0]);
  in.set_size(1, 45);
  for (int i{0}; i < 45; i++) {
    in[i] = i_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 44);
    parameterName[0] = '~';
    for (int i{0}; i < 43; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 44);
    for (int i{0}; i < 44; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 45; i++) {
      h_name[i] = i_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &h_name[0]);
    in.set_size(1, 45);
    for (int i{0}; i < 45; i++) {
      in[i] = i_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 44);
      parameterName[0] = '~';
      for (int i{0}; i < 43; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 44);
      for (int i{0}; i < 44; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_segmentation_buoys_clusterDist = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_segmentation_buoys_clusterDist);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_segmentation_buoys_clusterDist = 0.4;
  }
  //         %% get rosparam number
  for (int i{0}; i < 42; i++) {
    j_name[i] = k_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &j_name[0]);
  in.set_size(1, 42);
  for (int i{0}; i < 42; i++) {
    in[i] = k_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 41);
    parameterName[0] = '~';
    for (int i{0}; i < 40; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 41);
    for (int i{0}; i < 41; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 42; i++) {
      j_name[i] = k_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &j_name[0]);
    in.set_size(1, 42);
    for (int i{0}; i < 42; i++) {
      in[i] = k_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 41);
      parameterName[0] = '~';
      for (int i{0}; i < 40; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 41);
      for (int i{0}; i < 41; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_segmentation_buoys_planeTol = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_segmentation_buoys_planeTol);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_segmentation_buoys_planeTol = 0.02;
  }
  //         %% get rosparam number
  for (int i{0}; i < 49; i++) {
    l_name[i] = m_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &l_name[0]);
  in.set_size(1, 49);
  for (int i{0}; i < 49; i++) {
    in[i] = m_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 48);
    parameterName[0] = '~';
    for (int i{0}; i < 47; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 48);
    for (int i{0}; i < 48; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 49; i++) {
      l_name[i] = m_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &l_name[0]);
    in.set_size(1, 49);
    for (int i{0}; i < 49; i++) {
      in[i] = m_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 48);
      parameterName[0] = '~';
      for (int i{0}; i < 47; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 48);
      for (int i{0}; i < 48; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_segmentation_buoys_icpInlierRatio = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_segmentation_buoys_icpInlierRatio);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_segmentation_buoys_icpInlierRatio = 0.1;
  }
  //         %% get rosparam number
  for (int i{0}; i < 47; i++) {
    n_name[i] = o_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &n_name[0]);
  in.set_size(1, 47);
  for (int i{0}; i < 47; i++) {
    in[i] = o_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 46);
    parameterName[0] = '~';
    for (int i{0}; i < 45; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 46);
    for (int i{0}; i < 46; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 47; i++) {
      n_name[i] = o_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &n_name[0]);
    in.set_size(1, 47);
    for (int i{0}; i < 47; i++) {
      in[i] = o_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 46);
      parameterName[0] = '~';
      for (int i{0}; i < 45; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 46);
      for (int i{0}; i < 46; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_segmentation_buoys_zNormalThres = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_segmentation_buoys_zNormalThres);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_segmentation_buoys_zNormalThres = 0.2;
  }
  //         %% get rosparam number
  for (int i{0}; i < 47; i++) {
    n_name[i] = p_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &n_name[0]);
  in.set_size(1, 47);
  for (int i{0}; i < 47; i++) {
    in[i] = p_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 46);
    parameterName[0] = '~';
    for (int i{0}; i < 45; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 46);
    for (int i{0}; i < 46; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 47; i++) {
      n_name[i] = p_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &n_name[0]);
    in.set_size(1, 47);
    for (int i{0}; i < 47; i++) {
      in[i] = p_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 46);
      parameterName[0] = '~';
      for (int i{0}; i < 45; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 46);
      for (int i{0}; i < 46; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_segmentation_buoys_inPlaneThres = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_segmentation_buoys_inPlaneThres);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_segmentation_buoys_inPlaneThres = 0.4;
  }
  //         %% get rosparam number
  for (int i{0}; i < 41; i++) {
    name[i] = q_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                    &name[0]);
  in.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    in[i] = q_name[i];
  }
  if (coder::startsWith(in)) {
    parameterName.set_size(1, 40);
    parameterName[0] = '~';
    for (int i{0}; i < 39; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 40);
    for (int i{0}; i < 40; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    for (int i{0}; i < 41; i++) {
      name[i] = q_name[i];
    }
    std::mem_fn (&MATLABROSParameter::isValidPattern)(&lobj_1.ParameterHelper,
                                                      &name[0]);
    in.set_size(1, 41);
    for (int i{0}; i < 41; i++) {
      in[i] = q_name[i];
    }
    if (coder::startsWith(in)) {
      parameterName.set_size(1, 40);
      parameterName[0] = '~';
      for (int i{0}; i < 39; i++) {
        parameterName[i + 1] = in[i + 2];
      }
      in.set_size(1, 40);
      for (int i{0}; i < 40; i++) {
        in[i] = parameterName[i];
      }
    }
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_segmentation_buoys_minArea = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_segmentation_buoys_minArea);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_segmentation_buoys_minArea = 0.6;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    coder::ros::ParameterTree::canonicalizeName(&lobj_1, in);
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_segmentation_buoys_maxArea = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_segmentation_buoys_maxArea);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_segmentation_buoys_maxArea = 2.5;
  }
}

// End of code generation (RosNode.cpp)
