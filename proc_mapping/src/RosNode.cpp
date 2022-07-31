//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RosNode.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "RosNode.h"
#include "Buoys.h"
#include "Kdtree.h"
#include "ParameterTree.h"
#include "PointCloudBundler.h"
#include "Preprocessing.h"
#include "Publisher.h"
#include "Rate.h"
#include "SoundCloudBundler.h"
#include "Subscriber.h"
#include "Tables.h"
#include "combineVectorElements.h"
#include "dot.h"
#include "find.h"
#include "histcounts.h"
#include "mode.h"
#include "pccat.h"
#include "pcdenoise.h"
#include "pcdownsample.h"
#include "pcsegdist.h"
#include "pctransform.h"
#include "pointCloud.h"
#include "proc_mapping_data.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_types.h"
#include "quatinv.h"
#include "rigid3d.h"
#include "rt_nonfinite.h"
#include "sonar2NED.h"
#include "sonia_common_ObstacleInfoStruct.h"
#include "strcmp.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_posix_time.h"
#include "mlroscpp_param.h"
#include "mlroscpp_pub.h"
#include "mlroscpp_rate.h"
#include <cmath>
#include <functional>
#include <stdio.h>
#include <string.h>

// Function Definitions
//
// Preprocessing
//
// Arguments    : double *param_preprocessing_minIntensity
//                double *param_preprocessing_maxIntensity
//                double *param_preprocessing_minRange
//                double *param_preprocessing_maxRange
//                double *param_filter_sonar_general_boxSize
//                double *param_filter_sonar_histogram_filter_nBin
//                double *param_filter_sonar_histogram_filter_nBinsToFilterOut
//                double *param_filter_hydro_general_boxSize
//                double *param_filter_hydro_freqThreshold
//                double *param_filter_general_boxSize
//                h_struct_T *param_segmentation_buoys
//                i_struct_T *param_segmentation_tables
//                g_struct_T *param_segmentation_hydro
//                l_struct_T *param_parameters_hydro
//                m_struct_T *param_parameters_sonar
// Return Type  : void
//
void RosNode::getRosParams(
    double *param_preprocessing_minIntensity,
    double *param_preprocessing_maxIntensity,
    double *param_preprocessing_minRange, double *param_preprocessing_maxRange,
    double *param_filter_sonar_general_boxSize,
    double *param_filter_sonar_histogram_filter_nBin,
    double *param_filter_sonar_histogram_filter_nBinsToFilterOut,
    double *param_filter_hydro_general_boxSize,
    double *param_filter_hydro_freqThreshold,
    double *param_filter_general_boxSize, h_struct_T *param_segmentation_buoys,
    i_struct_T *param_segmentation_tables, g_struct_T *param_segmentation_hydro,
    l_struct_T *param_parameters_hydro, m_struct_T *param_parameters_sonar)
{
  coder::ros::ParameterTree lobj_1;
  coder::array<char, 2U> in;
  coder::array<char, 2U> parameterName;
  int loop_ub;
  bool nameExists;
  //  Filter
  //  General
  *param_filter_general_boxSize = 0.05;
  //  Histogram
  //  Segmentation
  //  Buoys
  //  Tables
  //  rad
  //  Hydro
  //  Parameters
  //  Hydro
  //  Sonar
  lobj_1.ParameterHelper = MATLABROSParameter();
  UNUSED_PARAM(lobj_1.ParameterHelper);
  // ROSPARAM Construct an instance of this class
  //    Detailed explanation goes here
  //  Preprocessing
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
    *param_preprocessing_minIntensity = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_preprocessing_minIntensity);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_preprocessing_minIntensity = 0.01;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::b_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    coder::ros::ParameterTree::b_canonicalizeName(&lobj_1, in);
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
  coder::ros::ParameterTree::c_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    coder::ros::ParameterTree::c_canonicalizeName(&lobj_1, in);
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
  coder::ros::ParameterTree::d_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    coder::ros::ParameterTree::d_canonicalizeName(&lobj_1, in);
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
  coder::ros::ParameterTree::e_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    coder::ros::ParameterTree::e_canonicalizeName(&lobj_1, in);
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (int i{0}; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    *param_filter_sonar_general_boxSize = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        param_filter_sonar_general_boxSize);
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_filter_sonar_general_boxSize = 0.05;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::f_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    *param_filter_hydro_general_boxSize = lobj_1.get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_filter_hydro_general_boxSize = 0.05;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::g_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    *param_filter_hydro_freqThreshold = lobj_1.b_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_filter_hydro_freqThreshold = 1000.0;
  }
  //  Histogram
  //         %% get rosparam number
  coder::ros::ParameterTree::h_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    *param_filter_sonar_histogram_filter_nBin = lobj_1.c_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_filter_sonar_histogram_filter_nBin = 100.0;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::i_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    *param_filter_sonar_histogram_filter_nBinsToFilterOut = lobj_1.d_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    *param_filter_sonar_histogram_filter_nBinsToFilterOut = 10.0;
  }
  //  Segmentation
  //  Buoys
  //         %% get rosparam number
  coder::ros::ParameterTree::j_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_buoys->clusterDist = lobj_1.e_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_buoys->clusterDist = 0.4;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::k_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_buoys->planeTol = lobj_1.f_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_buoys->planeTol = 0.05;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::l_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_buoys->icpInlierRatio = lobj_1.g_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_buoys->icpInlierRatio = 0.1;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::m_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_buoys->zNormalThres = lobj_1.h_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_buoys->zNormalThres = 0.2;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::n_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_buoys->inPlaneThres = lobj_1.i_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_buoys->inPlaneThres = 0.4;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::o_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_buoys->minArea = lobj_1.j_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_buoys->minArea = 0.6;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::p_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_buoys->maxArea = lobj_1.k_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_buoys->maxArea = 2.5;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::q_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_buoys->gap = lobj_1.l_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_buoys->gap = 0.025;
  }
  //  Tables
  //         %% get rosparam number
  coder::ros::ParameterTree::r_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_tables->clusterDist = lobj_1.m_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_tables->clusterDist = 0.5;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::s_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_tables->topAreaMin = lobj_1.n_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_tables->topAreaMin = 0.6;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::t_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_tables->topAreaMax = lobj_1.o_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_tables->topAreaMax = 2.0;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::u_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_tables->poseDepth = lobj_1.p_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_tables->poseDepth = 1.0;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::v_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_tables->maxBetweenDist = lobj_1.q_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_tables->maxBetweenDist = 4.0;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::w_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_tables->maxBetweenAngle = lobj_1.r_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_tables->maxBetweenAngle = 0.3;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::x_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_tables->squarenessRatio = lobj_1.s_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_tables->squarenessRatio = 0.3;
  }
  //  Hydro
  //         %% get rosparam number
  coder::ros::ParameterTree::y_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_segmentation_hydro->clusterDist = lobj_1.t_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_segmentation_hydro->clusterDist = 0.5;
  }
  //  Parameters
  //  Hydro
  //         %% get rosparam number
  coder::ros::ParameterTree::ab_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_parameters_hydro->pingerDepth = lobj_1.u_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_parameters_hydro->pingerDepth = 5.0;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::bb_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_parameters_hydro->translation.x = lobj_1.v_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_parameters_hydro->translation.x = 0.155;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::cb_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_parameters_hydro->translation.y = lobj_1.w_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_parameters_hydro->translation.y = 0.0;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::db_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_parameters_hydro->translation.z = lobj_1.x_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_parameters_hydro->translation.z = 0.118;
  }
  //  Sonar
  //         %% get rosparam number
  coder::ros::ParameterTree::eb_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_parameters_sonar->translation.x = lobj_1.y_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_parameters_sonar->translation.x = 0.358;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::fb_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_parameters_sonar->translation.y = lobj_1.ab_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_parameters_sonar->translation.y = 0.0;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::gb_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    param_parameters_sonar->translation.z = lobj_1.bb_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    param_parameters_sonar->translation.z = -0.118;
  }
}

//
// Arguments    : coder::ros::Rate *b_spin
//                SoundCloudBundler *iobj_0
//                PointCloudBundler *iobj_1
// Return Type  : void
//
void RosNode::spin(coder::ros::Rate *b_spin, SoundCloudBundler *iobj_0,
                   PointCloudBundler *iobj_1)
{
  static coder::f_pointCloud buoy1;
  static coder::f_pointCloud buoy2;
  static coder::g_pointCloud buoy;
  static const char b_cv[128]{
      '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\x07', '\x08',
      '\x09', '\x0a', '\x0b', '\x0c', '\x0d', '\x0e', '\x0f', '\x10', '\x11',
      '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a',
      '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',    '!',    '\"',   '#',
      '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',
      '-',    '.',    '/',    '0',    '1',    '2',    '3',    '4',    '5',
      '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',
      '?',    '@',    'A',    'B',    'C',    'D',    'E',    'F',    'G',
      'H',    'I',    'J',    'K',    'L',    'M',    'N',    'O',    'P',
      'Q',    'R',    'S',    'T',    'U',    'V',    'W',    'X',    'Y',
      'Z',    '[',    '\\',   ']',    '^',    '_',    '`',    'A',    'B',
      'C',    'D',    'E',    'F',    'G',    'H',    'I',    'J',    'K',
      'L',    'M',    'N',    'O',    'P',    'Q',    'R',    'S',    'T',
      'U',    'V',    'W',    'X',    'Y',    'Z',    '{',    '|',    '}',
      '~',    '\x7f'};
  static const char b_cv1[6]{'P', 'i', 'n', 'g', 'e', 'r'};
  Buoys b_buoys;
  Buoys buoys;
  Tables tables;
  coder::b_pointCloud lobj_3;
  coder::b_pointCloud *ptCloud;
  coder::d_pointCloud b_lobj_1;
  coder::d_pointCloud lobj_1;
  coder::f_pointCloud *ptClouds;
  coder::pointCloud b_filt;
  coder::pointCloud b_rawPT;
  coder::pointCloud filt;
  coder::pointCloud hydroPt;
  coder::pointCloud rawPT;
  coder::pointCloud *ptCloudOut;
  coder::rigid3d T1;
  coder::rigid3d T2;
  coder::ros::Subscriber *b_sub;
  coder::ros::b_Subscriber *c_sub;
  coder::ros::c_Subscriber *d_sub;
  coder::ros::d_Subscriber *sub;
  coder::ros::e_Subscriber *e_sub;
  coder::vision::internal::codegen::Kdtree b_lobj_2;
  coder::vision::internal::codegen::Kdtree c_lobj_1;
  coder::vision::internal::codegen::Kdtree c_lobj_2;
  coder::vision::internal::codegen::Kdtree lobj_2;
  coder::vision::internal::codegen::Kdtree lobj_4;
  coder::vision::internal::codegen::Kdtree lobj_5;
  coder::array<double, 2U> a__1;
  coder::array<double, 2U> b_r;
  coder::array<double, 2U> bundle;
  coder::array<double, 2U> c_bundle;
  coder::array<double, 2U> d_bundle;
  coder::array<double, 2U> edges;
  coder::array<double, 2U> nv;
  coder::array<double, 1U> b_bundle;
  coder::array<double, 1U> intensity;
  coder::array<unsigned int, 1U> b_labels;
  coder::array<unsigned int, 1U> labels;
  coder::array<int, 1U> r;
  coder::array<int, 1U> r1;
  coder::array<char, 2U> b_this;
  coder::array<unsigned char, 2U> c;
  coder::array<char, 2U> switch_expression;
  coder::array<char, 2U> t9_Data;
  coder::array<bool, 1U> logic;
  geometry_msgs_PoseWithCovarianceStruct_T unusedExpr;
  sonia_common_ObstacleInfoStruct_T obstacle;
  double b_quat[4];
  double quat[4];
  double pos[3];
  double expl_temp;
  double t11_Pose_Pose_Orientation_X;
  double t12_Pose_Pose_Orientation_Y;
  double z;
  int i;
  unsigned short hydroMsg_Frequency;
  unsigned short hydroMsg_Snr;
  filt.matlabCodegenIsDeleted = true;
  b_filt.matlabCodegenIsDeleted = true;
  lobj_3.matlabCodegenIsDeleted = true;
  //         %% ROS Spinfilt
  MATLABRate_reset(b_spin->RateHelper);
  coder::tic(&expl_temp, &z);
  printf("INFO : proc mapping : Node is started. \n");
  fflush(stdout);
  //  Instances
  //         %% PointCloudBundler Constructor
  sub = iobj_1->_pobj0.init();
  iobj_1->mPoseSub = sub;
  iobj_1->mParam = param;
  //  Graphics functions
  iobj_1->mLastBundleState = false;
  iobj_1->mBundle.set_size(3, 4);
  for (i = 0; i < 12; i++) {
    iobj_1->mBundle[i] = 0.0;
  }
  iobj_1->mBundle.set_size(1, 4);
  iobj_1->mBundle[0] = 0.0;
  iobj_1->mBundle[1] = 0.0;
  iobj_1->mBundle[2] = 0.0;
  iobj_1->mBundle[3] = 0.0;
  //  Subscribers
  b_sub = iobj_1->_pobj4.init();
  iobj_1->mStartSub = b_sub;
  c_sub = iobj_1->_pobj3.init();
  iobj_1->mStopSub = c_sub;
  d_sub = iobj_1->_pobj2.init();
  iobj_1->mClearBundleSub = d_sub;
  e_sub = iobj_1->_pobj1.init();
  iobj_1->mSonarSub = e_sub;
  //  ROS Subscribers
  //         %% Preprocessing Constructor
  iobj_1->mPreprocessing.param = param.preprocessing;
  mPtBundler = iobj_1;
  mScBundler = iobj_0->init(&param);
  while (1) {
    double center[3];
    double rho;
    int end;
    int loop_ub;
    bool guard1{false};
    bool out;
    //  PointCloud Bundler
    if (!iobj_1->step()) {
      printf("INFO : proc mapping : Not bundling. \n");
      fflush(stdout);
      bundle.set_size(mPtBundler->mBundle.size(0), 4);
      loop_ub = mPtBundler->mBundle.size(0) * 4;
      for (i = 0; i < loop_ub; i++) {
        bundle[i] = mPtBundler->mBundle[i];
      }
      if (bundle.size(0) > 1) {
        //  Create and filter pointcloud form bundle
        //  Histogram filter
        //  Filter using histogram bins egdes, Only work for one column for 1
        //  column at this time, use plot = true to plot the histogram.
        //  Return the fitered Pointcloud
        loop_ub = bundle.size(0) - 2;
        b_bundle.set_size(bundle.size(0) - 1);
        for (i = 0; i <= loop_ub; i++) {
          b_bundle[i] = bundle[(i + bundle.size(0) * 3) + 1];
        }
        coder::histcounts(b_bundle, param.filter.sonar.histogram_filter.nBin,
                          a__1, edges);
        // remove 0,0,0
        expl_temp =
            edges[static_cast<int>(
                      param.filter.sonar.histogram_filter.nBinsToFilterOut +
                      1.0) -
                  1];
        loop_ub = bundle.size(0);
        logic.set_size(bundle.size(0));
        for (i = 0; i < loop_ub; i++) {
          logic[i] = (bundle[i + bundle.size(0) * 3] > expl_temp);
        }
        end = logic.size(0) - 1;
        loop_ub = 0;
        for (i = 0; i <= end; i++) {
          if (logic[i]) {
            loop_ub++;
          }
        }
        r.set_size(loop_ub);
        loop_ub = 0;
        for (i = 0; i <= end; i++) {
          if (logic[i]) {
            r[loop_ub] = i + 1;
            loop_ub++;
          }
        }
        d_bundle.set_size(r.size(0), 4);
        loop_ub = r.size(0);
        for (i = 0; i < 4; i++) {
          for (end = 0; end < loop_ub; end++) {
            d_bundle[end + d_bundle.size(0) * i] =
                bundle[(r[end] + bundle.size(0) * i) - 1];
          }
        }
        bundle.set_size(d_bundle.size(0), 4);
        loop_ub = d_bundle.size(0) * 4;
        for (i = 0; i < loop_ub; i++) {
          bundle[i] = d_bundle[i];
        }
        //  3e arg : optional bool debug graph default = false
        //  General filter
        b_filt.matlabCodegenDestructor();
        //  Create the point cloud and apply denoise filter plus a downsample.
        b_rawPT.matlabCodegenIsDeleted = true;
        b_lobj_1.matlabCodegenIsDeleted = true;
        loop_ub = bundle.size(0);
        c_bundle.set_size(bundle.size(0), 3);
        for (i = 0; i < 3; i++) {
          for (end = 0; end < loop_ub; end++) {
            c_bundle[end + c_bundle.size(0) * i] =
                bundle[end + bundle.size(0) * i];
          }
        }
        loop_ub = bundle.size(0);
        b_bundle.set_size(bundle.size(0));
        for (i = 0; i < loop_ub; i++) {
          b_bundle[i] = bundle[i + bundle.size(0) * 3];
        }
        b_rawPT.init(c_bundle, b_bundle, &b_lobj_2);
        coder::pcdenoise(coder::pcdownsample(
                             &b_rawPT, param.filter.general.boxSize, &b_lobj_1),
                         &lobj_4, &b_filt);
        b_lobj_1.matlabCodegenDestructor();
        b_rawPT.matlabCodegenDestructor();
        //         %% Getters / Setters
        mPtBundler->mStartSub->get_LatestMessage(b_this);
        mPtBundler->mStartSub->get_LatestMessage(t9_Data);
        switch_expression.set_size(1, t9_Data.size(1));
        i = t9_Data.size(1);
        for (loop_ub = 0; loop_ub < i; loop_ub++) {
          switch_expression[loop_ub] =
              b_cv[static_cast<unsigned char>(t9_Data[loop_ub]) & 127];
        }
        if (coder::internal::b_strcmp(switch_expression)) {
          loop_ub = 0;
        } else if (coder::internal::c_strcmp(switch_expression)) {
          loop_ub = 1;
        } else {
          loop_ub = -1;
        }
        switch (loop_ub) {
        case 0: {
          lobj_3.matlabCodegenDestructor();
          buoys.param = param.segmentation.buoys;
          //  BUOYS Construct an instance of this class
          //    Detailed explanation goes here
          buoy.matlabCodegenIsDeleted = true;
          buoy2.matlabCodegenIsDeleted = true;
          buoy1.matlabCodegenIsDeleted = true;
          buoys.filteredPT = &b_filt;
          loop_ub = b_filt.Location.size(0);
          buoys.PTlabels.set_size(1, loop_ub);
          for (i = 0; i < loop_ub; i++) {
            buoys.PTlabels[i] = 0U;
          }
          buoy.init();
          //  apply offset gaps on  pannel
          center[0] = 0.0;
          center[1] = param.segmentation.buoys.gap / 2.0;
          center[2] = 0.0;
          T1.init(center);
          center[0] = 0.0;
          center[1] = -param.segmentation.buoys.gap / 2.0;
          center[2] = 0.0;
          T2.init(center);
          coder::pctransform(&buoy, &T1, &buoy1);
          coder::pctransform(&buoy, &T2, &buoy2);
          ptClouds = buoy1.horzcat(&buoy2);
          ptCloud = coder::vision::internal::codegen::pc::pccat(
              ptClouds, &c_lobj_2, &lobj_3);
          buoys.buoyPT = ptCloud;
          loop_ub = ptCloud->Location.size(0);
          ptCloud->Intensity.set_size(loop_ub, 1);
          for (i = 0; i < loop_ub; i++) {
            ptCloud->Intensity[i] = 0.1F;
          }
          loop_ub = buoys.buoyPT->Location.size(0);
          ptCloud->Normal.set_size(loop_ub, 3);
          loop_ub *= 3;
          for (i = 0; i < loop_ub; i++) {
            ptCloud->Normal[i] = 0.0F;
          }
          double b_center[7];
          buoy1.matlabCodegenDestructor();
          buoy2.matlabCodegenDestructor();
          buoy.matlabCodegenDestructor();
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          expl_temp = unusedExpr.Pose.Position.X;
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          z = unusedExpr.Pose.Position.Y;
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          center[2] = unusedExpr.Pose.Position.Z;
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          rho = unusedExpr.Pose.Orientation.W;
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          t11_Pose_Pose_Orientation_X = unusedExpr.Pose.Orientation.X;
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          t12_Pose_Pose_Orientation_Y = unusedExpr.Pose.Orientation.Y;
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          b_center[0] = expl_temp;
          b_center[1] = z;
          b_center[2] = center[2];
          b_center[3] = rho;
          b_center[4] = t11_Pose_Pose_Orientation_X;
          b_center[5] = t12_Pose_Pose_Orientation_Y;
          b_center[6] = unusedExpr.Pose.Orientation.Z;
          b_buoys = buoys;
          b_buoys.SegementByAtribute(
              b_center, *(sonia_common_ObstacleInfoStruct_T(*)[2]) &
                            obstacleArray.Obstacles[1]);
          MATLABPUBLISHER_publish(obstacleArrayPublisher->PublisherHelper,
                                  &obstacleArray);
        } break;
        case 1:
          tables.param = param.segmentation.tables;
          //  BUOYS Construct an instance of this class
          //    Detailed explanation goes here
          tables.filteredPT = &b_filt;
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          tables.SegementByAtribute(&obstacleArray.Obstacles[6]);
          MATLABPUBLISHER_publish(obstacleArrayPublisher->PublisherHelper,
                                  &obstacleArray);
          break;
        }
      }
    } else {
      //  fprintf('INFO : proc mapping : sonar : Bundling or waiting. \n');
    }
    //  SoundCloud Bundler
    //  Verifiy if we just stop the record.
    //         %% Step function
    guard1 = false;
    if (mScBundler->mLastBundleState) {
      //  Initial variables
      //  GET
      out = b_bundleStarted;
      if (!out) {
        //  Record finished.
        mScBundler->mLastBundleState = false;
        printf("INFO : proc mapping : hydro : Record finished. \n");
        fflush(stdout);
        bundle.set_size(mScBundler->mBundle.size(0), 4);
        loop_ub = mScBundler->mBundle.size(0) * 4;
        for (i = 0; i < loop_ub; i++) {
          bundle[i] = mScBundler->mBundle[i];
        }
        if (bundle.size(0) > 1) {
          unsigned int b_index;
          //  Filter the hydrophone point cloud.
          filt.matlabCodegenDestructor();
          //  Create the point cloud and apply denoise filter plus a downsample.
          rawPT.matlabCodegenIsDeleted = true;
          lobj_1.matlabCodegenIsDeleted = true;
          loop_ub = bundle.size(0);
          c_bundle.set_size(bundle.size(0), 3);
          for (i = 0; i < 3; i++) {
            for (end = 0; end < loop_ub; end++) {
              c_bundle[end + c_bundle.size(0) * i] =
                  bundle[end + bundle.size(0) * i];
            }
          }
          loop_ub = bundle.size(0);
          b_bundle.set_size(bundle.size(0));
          for (i = 0; i < loop_ub; i++) {
            b_bundle[i] = bundle[i + bundle.size(0) * 3];
          }
          rawPT.init(c_bundle, b_bundle, &lobj_2);
          coder::pcdenoise(
              coder::pcdownsample(&rawPT, param.filter.hydro.general.boxSize,
                                  &lobj_1),
              &lobj_5, &filt);
          lobj_1.matlabCodegenDestructor();
          rawPT.matlabCodegenDestructor();
          //  Segment the point cloud and find the center.
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mPtBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          //  BUOYS Construct an instance of this class
          //    Detailed explanation goes here
          hydroPt.matlabCodegenIsDeleted = true;
          coder::pcsegdist(&filt, param.segmentation.hydro.clusterDist, labels,
                           &expl_temp);
          b_labels.set_size(labels.size(0));
          loop_ub = labels.size(0);
          for (i = 0; i < loop_ub; i++) {
            b_labels[i] = labels[i];
          }
          //  Get the biggest point cluster.
          b_index = coder::arraymode(labels);
          end = labels.size(0);
          for (i = 0; i < end; i++) {
            if (labels[i] != b_index) {
              b_labels[i] = 0U;
            }
          }
          end = b_labels.size(0);
          for (i = 0; i < end; i++) {
            if (b_labels[i] == b_index) {
              b_labels[i] = 1U;
            }
          }
          //  Extract the cluster.
          logic.set_size(b_labels.size(0));
          loop_ub = b_labels.size(0);
          for (i = 0; i < loop_ub; i++) {
            logic[i] = (b_labels[i] != 0U);
          }
          coder::b_eml_find(logic, r1);
          b_bundle.set_size(r1.size(0));
          loop_ub = r1.size(0);
          for (i = 0; i < loop_ub; i++) {
            b_bundle[i] = r1[i];
          }
          filt.subsetImpl(b_bundle, c_bundle, c, nv, intensity, b_r);
          ptCloudOut = hydroPt.init(c_bundle, c, nv, intensity, &c_lobj_1);
          ptCloudOut->RangeData.set_size(b_r.size(0), b_r.size(1));
          loop_ub = b_r.size(0) * b_r.size(1);
          for (i = 0; i < loop_ub; i++) {
            ptCloudOut->RangeData[i] = b_r[i];
          }
          //  Get the center of this cluster.
          c_bundle.set_size(hydroPt.Location.size(0), 3);
          loop_ub = hydroPt.Location.size(0) * 3;
          for (i = 0; i < loop_ub; i++) {
            c_bundle[i] = hydroPt.Location[i];
          }
          coder::combineVectorElements(c_bundle, center);
          center[0] /= static_cast<double>(c_bundle.size(0));
          center[1] /= static_cast<double>(c_bundle.size(0));
          center[2] /= static_cast<double>(c_bundle.size(0));
          sonia_common_ObstacleInfoStruct(&obstacle);
          obstacle.IsValid = true;
          obstacle.Name.set_size(1, 6);
          for (i = 0; i < 6; i++) {
            obstacle.Name[i] = b_cv1[i];
          }
          obstacle.Confidence = 100.0F;
          obstacle.Pose.Position.X = center[0];
          obstacle.Pose.Position.Y = center[1];
          obstacle.Pose.Position.Z = center[2];
          obstacle.Pose.Orientation.W = 1.0;
          obstacle.Pose.Orientation.X = 0.0;
          obstacle.Pose.Orientation.Y = 0.0;
          obstacle.Pose.Orientation.Z = 0.0;
          hydroPt.matlabCodegenDestructor();
          obstacleArray.Obstacles[5] = obstacle;
          MATLABPUBLISHER_publish(obstacleArrayPublisher->PublisherHelper,
                                  &obstacleArray);
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      //  Recording or waiting.
      //  Initial variables
      //  GET
      out = newHydroMsg;
      if (out) {
        //  Initial variables
        //  GET
        out = b_bundleStarted;
        if (out) {
          unsigned short t8_Data;
          mScBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          mScBundler->mHydroSub->get_LatestMessage(
              &t12_Pose_Pose_Orientation_Y, &t11_Pose_Pose_Orientation_X,
              &hydroMsg_Frequency, &hydroMsg_Snr);
          mScBundler->mPoseSub->get_LatestMessage(&unusedExpr);
          //  Adding to the point cloud.
          //         %% Getters / Setters
          mScBundler->mStartSub->get_LatestMessage();
          t8_Data = mScBundler->mStartSub->get_LatestMessage();
          //  Thresholds for a ping to be validate.
          if ((!(hydroMsg_Frequency >
                 static_cast<double>(t8_Data) +
                     mScBundler->mParam.filter.hydro.freqThreshold)) &&
              (!(hydroMsg_Frequency <
                 static_cast<double>(t8_Data) -
                     mScBundler->mParam.filter.hydro.freqThreshold))) {
            //  Getting the sub pose.
            pos[0] = unusedExpr.Pose.Position.X;
            pos[1] = unusedExpr.Pose.Position.Y;
            pos[2] = unusedExpr.Pose.Position.Z;
            quat[0] = unusedExpr.Pose.Orientation.W;
            quat[1] = unusedExpr.Pose.Orientation.X;
            quat[2] = unusedExpr.Pose.Orientation.Y;
            quat[3] = unusedExpr.Pose.Orientation.Z;
            // fix
            coder::quatinv(quat);
            b_quat[0] = unusedExpr.Pose.Orientation.W;
            b_quat[1] = unusedExpr.Pose.Orientation.X;
            b_quat[2] = unusedExpr.Pose.Orientation.Y;
            b_quat[3] = unusedExpr.Pose.Orientation.Z;
            //  Trouver la valeur de z
            center[0] = mScBundler->mHydroPose[0];
            center[1] = mScBundler->mHydroPose[1];
            center[2] = mScBundler->mHydroPose[2];
            coder::quatinv(b_quat);
            // =================================================================
            //  Fonction qui tourne un vecteur selon un quaternion.
            //  quaternion partie scalaire
            //  quaternion partie vectoriel
            //  QuatRotate n'est pas compilable
            expl_temp = 2.0 * coder::dot(*(double(*)[3]) & b_quat[1], center);
            z = b_quat[0] * b_quat[0] - coder::dot(*(double(*)[3]) & b_quat[1],
                                                   *(double(*)[3]) & b_quat[1]);
            z = (mScBundler->mParam.parameters.hydro.pingerDepth -
                 unusedExpr.Pose.Position.Z) +
                ((expl_temp * b_quat[3] + z * center[2]) +
                 2.0 * b_quat[0] *
                     (b_quat[1] * center[1] - center[0] * b_quat[2]));
            rho = z / std::cos(t11_Pose_Pose_Orientation_X);
            expl_temp = std::sin(t11_Pose_Pose_Orientation_X);
            // apply puck rotation
            //  hydro = quatrotate(eul2quat(deg2rad([-150,0,0]),'ZYX'),hydro.');
            center[0] =
                rho *
                std::cos(t12_Pose_Pose_Orientation_Y + 3.6651914291880923) *
                expl_temp;
            center[1] =
                rho *
                std::sin(t12_Pose_Pose_Orientation_Y + 3.6651914291880923) *
                expl_temp;
            center[2] = z;
            sonar2NED(pos, quat, mScBundler->mHydroPose, center, b_quat);
            mScBundler->i++;
            bundle.set_size(mScBundler->mBundle.size(0) + 1, 4);
            loop_ub = mScBundler->mBundle.size(0);
            for (i = 0; i < 4; i++) {
              for (end = 0; end < loop_ub; end++) {
                bundle[end + bundle.size(0) * i] =
                    mScBundler->mBundle[end + mScBundler->mBundle.size(0) * i];
              }
            }
            bundle[mScBundler->mBundle.size(0)] = b_quat[0];
            bundle[mScBundler->mBundle.size(0) + bundle.size(0)] = b_quat[1];
            bundle[mScBundler->mBundle.size(0) + bundle.size(0) * 2] =
                b_quat[2];
            bundle[mScBundler->mBundle.size(0) + bundle.size(0) * 3] =
                hydroMsg_Snr;
            mScBundler->mBundle.set_size(bundle.size(0), 4);
            loop_ub = bundle.size(0) * 4;
            for (i = 0; i < loop_ub; i++) {
              mScBundler->mBundle[i] = bundle[i];
            }
          }
          //  Initial variables
          //  SET
          newHydroMsg = false;
        }
      }
      //  Clear the buffer if requested.
      //  Initial variables
      //  GET
      out = b_newClearBundleMsg;
      if (out) {
        printf("INFO : proc mapping : hydro : Clearing the sonar bundle \n");
        fflush(stdout);
        mScBundler->mBundle.set_size(1, 4);
        mScBundler->mBundle[0] = 0.0;
        mScBundler->mBundle[1] = 0.0;
        mScBundler->mBundle[2] = 0.0;
        mScBundler->mBundle[3] = 0.0;
        //  Initial variables
        //  SET
        b_newClearBundleMsg = false;
      }
      //  Initial variables
      //  GET
      out = b_bundleStarted;
      mScBundler->mLastBundleState = out;
      //  fprintf('INFO : proc mapping : hydro : Bundling or waiting. \n');
    }
    //  Update ROS parameters
    rosParamCounter++;
    if (rosParamCounter >= rate * paramUpdateRate) {
      rosParamCounter = 0.0;
      RosNode::getRosParams(
          &param.preprocessing.minIntensity, &param.preprocessing.maxIntensity,
          &param.preprocessing.minRange, &param.preprocessing.maxRange,
          &param.filter.sonar.general.boxSize,
          &param.filter.sonar.histogram_filter.nBin,
          &param.filter.sonar.histogram_filter.nBinsToFilterOut,
          &param.filter.hydro.general.boxSize,
          &param.filter.hydro.freqThreshold, &param.filter.general.boxSize,
          &param.segmentation.buoys, &param.segmentation.tables,
          &param.segmentation.hydro, &param.parameters.hydro,
          &param.parameters.sonar);
      mPtBundler->mParam = param;
      iobj_1->mPreprocessing.param = param.preprocessing;
      mScBundler->mParam = param;
    }
    MATLABRate_sleep(b_spin->RateHelper);
    coder::toc(b_spin->PreviousPeriod.tv_sec, b_spin->PreviousPeriod.tv_nsec);
    coder::tic(&b_spin->PreviousPeriod.tv_sec, &b_spin->PreviousPeriod.tv_nsec);
  }
}

//
// File trailer for RosNode.cpp
//
// [EOF]
//
