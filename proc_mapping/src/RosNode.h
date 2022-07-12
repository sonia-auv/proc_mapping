//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// RosNode.h
//
// Code generation for function 'RosNode'
//

#ifndef ROSNODE_H
#define ROSNODE_H

// Include files
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

// Type Definitions
class RosNode {
public:
  static void getRosParams(double *param_preprocessing_minIntensity,
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
                           double *param_segmentation_buoys_maxArea);
  f_struct_T param;
  double counter;
  sonia_common_ObstacleArrayStruct_T obstacleArray;

private:
  coder::ros::Publisher *outputCloudPublisher;
  coder::ros::b_Publisher *obstacleArrayPublisher;
  PointCloudBundler *mPtBundler;
  double paramUpdateRate;
  double rate;
};

#endif
// End of code generation (RosNode.h)
