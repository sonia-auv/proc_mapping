//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SoundCloudBundler.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "SoundCloudBundler.h"
#include "Subscriber.h"
#include "proc_mapping_data.h"
#include "proc_mapping_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const o_struct_T *param
// Return Type  : SoundCloudBundler *
//
SoundCloudBundler *SoundCloudBundler::init(const o_struct_T *param)
{
  SoundCloudBundler *this_;
  coder::ros::b_Subscriber *d_sub;
  coder::ros::c_Subscriber *e_sub;
  coder::ros::d_Subscriber *sub;
  coder::ros::f_Subscriber *c_sub;
  coder::ros::g_Subscriber *b_sub;
  this_ = this;
  //         %% SoundCloudBundler Constructor
  sub = this_->_pobj0.init();
  this_->mPoseSub = sub;
  this_->mParam = *param;
  //  Graphics functions
  this_->mLastBundleState = false;
  //  Gettho way to enable dynamic allocation
  this_->mBundle.set_size(3, 4);
  for (int b_i{0}; b_i < 12; b_i++) {
    this_->mBundle[b_i] = 0.0;
  }
  this_->mBundle.set_size(1, 4);
  this_->mBundle[0] = 0.0;
  this_->mBundle[1] = 0.0;
  this_->mBundle[2] = 0.0;
  this_->mBundle[3] = 0.0;
  this_->mHydroPose[0] = param->parameters.hydro.translation.x;
  this_->mHydroPose[1] = param->parameters.hydro.translation.y;
  this_->mHydroPose[2] = param->parameters.hydro.translation.z;
  //  Subscribers
  b_sub = this_->_pobj4.init();
  this_->mHydroSub = b_sub;
  c_sub = this_->_pobj3.init();
  this_->mStartSub = c_sub;
  d_sub = this_->_pobj2.b_init();
  this_->mStopSub = d_sub;
  e_sub = this_->_pobj1.b_init();
  this_->mClearBundleSub = e_sub;
  this_->i = 1.0;
  return this_;
}

//
// Arguments    : void
// Return Type  : void
//
void SoundCloudBundler::persistentDataStore_init()
{
  newHydroMsg = false;
  b_bundleStarted = false;
  b_newClearBundleMsg = false;
}

//
// File trailer for SoundCloudBundler.cpp
//
// [EOF]
//
