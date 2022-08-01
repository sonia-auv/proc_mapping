//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PointCloudBundler.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef POINTCLOUDBUNDLER_H
#define POINTCLOUDBUNDLER_H

// Include Files
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
  coder::ros::Subscriber *mStartSub;
  coder::ros::b_Subscriber *mStopSub;
  coder::ros::c_Subscriber *mClearBundleSub;
  coder::ros::d_Subscriber *mPoseSub;
  bool mLastBundleState;
  o_struct_T mParam;
  coder::array<double, 2U> mBundle;
  Preprocessing mPreprocessing;
  coder::ros::e_Subscriber *mSonarSub;
  coder::ros::d_Subscriber _pobj0;
  coder::ros::e_Subscriber _pobj1;
  coder::ros::c_Subscriber _pobj2;
  coder::ros::b_Subscriber _pobj3;
  coder::ros::Subscriber _pobj4;
};

#endif
//
// File trailer for PointCloudBundler.h
//
// [EOF]
//
