//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PointCloudBundler.h
//
// Code generation for function 'PointCloudBundler'
//

#ifndef POINTCLOUDBUNDLER_H
#define POINTCLOUDBUNDLER_H

// Include files
#include "Subscriber.h"
#include "pointCloud.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class PointCloudBundler {
public:
  void matlabCodegenDestructor();
  static void persistentDataStore_init();
  ~PointCloudBundler();
  PointCloudBundler();
  bool matlabCodegenIsDeleted;
  coder::b_pointCloud bigCloud;
  coder::array<double, 2U> bundle;
  coder::ros::Subscriber *startStopSub;
  coder::ros::b_Subscriber *poseSub;
  coder::ros::c_Subscriber *sonarSub;
  bool lastBundleState;
  coder::ros::c_Subscriber _pobj0;
  coder::ros::b_Subscriber _pobj1;
  coder::ros::Subscriber _pobj2;
};

// Function Declarations
void b_binary_expand_op(coder::array<double, 1U> &in1,
                        const coder::array<double, 1U> &in2,
                        const coder::array<double, 1U> &in3);

void binary_expand_op(coder::array<bool, 1U> &in1,
                      const coder::array<double, 2U> &in2,
                      const coder::array<double, 1U> &in3);

#endif
// End of code generation (PointCloudBundler.h)
