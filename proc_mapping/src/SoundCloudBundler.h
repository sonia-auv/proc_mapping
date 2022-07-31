//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SoundCloudBundler.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef SOUNDCLOUDBUNDLER_H
#define SOUNDCLOUDBUNDLER_H

// Include Files
#include "Subscriber.h"
#include "proc_mapping_internal_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class SoundCloudBundler {
public:
  SoundCloudBundler *init(const o_struct_T *param);
  static void persistentDataStore_init();
  coder::ros::f_Subscriber *mStartSub;
  coder::ros::d_Subscriber *mPoseSub;
  bool mLastBundleState;
  o_struct_T mParam;
  coder::array<double, 2U> mBundle;
  coder::ros::g_Subscriber *mHydroSub;
  double mHydroPose[3];
  double i;
  coder::ros::d_Subscriber _pobj0;
  coder::ros::c_Subscriber _pobj1;
  coder::ros::b_Subscriber _pobj2;
  coder::ros::f_Subscriber _pobj3;
  coder::ros::g_Subscriber _pobj4;

protected:
  coder::ros::b_Subscriber *mStopSub;
  coder::ros::c_Subscriber *mClearBundleSub;
};

#endif
//
// File trailer for SoundCloudBundler.h
//
// [EOF]
//
