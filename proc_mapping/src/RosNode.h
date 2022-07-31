//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RosNode.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef ROSNODE_H
#define ROSNODE_H

// Include Files
#include "proc_mapping_internal_types.h"
#include "proc_mapping_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace ros {
class Publisher;

class b_Publisher;

} // namespace ros
} // namespace coder
class PointCloudBundler;

class SoundCloudBundler;

namespace coder {
namespace ros {
class Rate;

}
} // namespace coder

// Type Definitions
class RosNode {
public:
  static void
  getRosParams(double *param_preprocessing_minIntensity,
               double *param_preprocessing_maxIntensity,
               double *param_preprocessing_minRange,
               double *param_preprocessing_maxRange,
               double *param_filter_sonar_general_boxSize,
               double *param_filter_sonar_histogram_filter_nBin,
               double *param_filter_sonar_histogram_filter_nBinsToFilterOut,
               double *param_filter_hydro_general_boxSize,
               double *param_filter_hydro_freqThreshold,
               double *param_filter_general_boxSize,
               h_struct_T *param_segmentation_buoys,
               i_struct_T *param_segmentation_tables,
               g_struct_T *param_segmentation_hydro,
               l_struct_T *param_parameters_hydro,
               m_struct_T *param_parameters_sonar);
  void spin(coder::ros::Rate *b_spin, SoundCloudBundler *iobj_0,
            PointCloudBundler *iobj_1);
  coder::ros::Publisher *outputCloudPublisher;
  coder::ros::b_Publisher *obstacleArrayPublisher;
  o_struct_T param;
  double paramUpdateRate;
  double rosParamCounter;
  double rate;
  sonia_common_ObstacleArrayStruct_T obstacleArray;

private:
  PointCloudBundler *mPtBundler;
  SoundCloudBundler *mScBundler;
};

#endif
//
// File trailer for RosNode.h
//
// [EOF]
//
