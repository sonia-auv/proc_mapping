//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: proc_mapping.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "proc_mapping.h"
#include "PointCloudBundler.h"
#include "Publisher.h"
#include "Rate.h"
#include "RosNode.h"
#include "SoundCloudBundler.h"
#include "proc_mapping_data.h"
#include "proc_mapping_initialize.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sonia_common_ObstacleArrayStruct.h"
#include "sonia_common_ObstacleInfoStruct.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void proc_mapping()
{
  PointCloudBundler lobj_2;
  RosNode rosNode;
  SoundCloudBundler lobj_1;
  coder::ros::Publisher lobj_4;
  coder::ros::Publisher *pub;
  coder::ros::Rate r;
  coder::ros::b_Publisher lobj_3;
  coder::ros::b_Publisher *b_pub;
  sonia_common_ObstacleInfoStruct_T y[10];
  sonia_common_ObstacleInfoStruct_T obstacle;
  if (!isInitialized_proc_mapping) {
    proc_mapping_initialize();
  }
  //  Variables
  r.init();
  //  Proc_mapping startup
  //  ROS Publishers
  //         %% ROS Node constructor
  pub = lobj_4.init();
  rosNode.outputCloudPublisher = pub;
  b_pub = lobj_3.init();
  rosNode.obstacleArrayPublisher = b_pub;
  sonia_common_ObstacleInfoStruct(&obstacle);
  sonia_common_ObstacleArrayStruct(&rosNode.obstacleArray);
  rosNode.obstacleArray.Obstacles.set_size(10);
  for (int j{0}; j < 10; j++) {
    y[j] = obstacle;
    rosNode.obstacleArray.Obstacles[j] = y[j];
  }
  //  ROS parameters
  RosNode::getRosParams(
      &rosNode.param.preprocessing.minIntensity,
      &rosNode.param.preprocessing.maxIntensity,
      &rosNode.param.preprocessing.minRange,
      &rosNode.param.preprocessing.maxRange,
      &rosNode.param.filter.sonar.general.boxSize,
      &rosNode.param.filter.sonar.histogram_filter.nBin,
      &rosNode.param.filter.sonar.histogram_filter.nBinsToFilterOut,
      &rosNode.param.filter.hydro.general.boxSize,
      &rosNode.param.filter.hydro.freqThreshold,
      &rosNode.param.filter.general.boxSize, &rosNode.param.segmentation.buoys,
      &rosNode.param.segmentation.tables, &rosNode.param.segmentation.hydro,
      &rosNode.param.parameters.hydro, &rosNode.param.parameters.sonar);
  rosNode.paramUpdateRate = 2.0;
  //  seconds
  rosNode.rosParamCounter = 0.0;
  rosNode.rate = 20.0;
  rosNode.spin(&r, &lobj_1, &lobj_2);
}

//
// File trailer for proc_mapping.cpp
//
// [EOF]
//
