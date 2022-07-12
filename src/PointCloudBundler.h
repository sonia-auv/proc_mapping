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
#include "Preprocessing.h"
#include "Subscriber.h"
#include "proc_mapping_internal_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class PointCloudBundler {
public:
  bool step();
  static void persistentDataStore_init();
  coder::array<double, 2U> mBundle;
  Preprocessing mPreprocessing;
  coder::ros::Subscriber *mStartSub;
  coder::ros::b_Subscriber *mStopSub;
  coder::ros::c_Subscriber *mClearBundleSub;
  coder::ros::d_Subscriber *mPoseSub;
  coder::ros::e_Subscriber *mSonarSub;
  coder::ros::f_Subscriber *mImageSub;
  bool mLastBundleState;
  f_struct_T param;
  coder::ros::f_Subscriber _pobj0;
  coder::ros::e_Subscriber _pobj1;
  coder::ros::d_Subscriber _pobj2;
  coder::ros::c_Subscriber _pobj3;
  coder::ros::b_Subscriber _pobj4;
  coder::ros::Subscriber _pobj5;
};

#endif
// End of code generation (PointCloudBundler.h)