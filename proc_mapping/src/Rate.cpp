//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Rate.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "Rate.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "coder_posix_time.h"
#include "mlroscpp_rate.h"
#include "ros/ros.h"
#include <string.h>

// Function Definitions
//
// Arguments    : void
// Return Type  : Rate *
//
namespace coder {
namespace ros {
Rate *Rate::init()
{
  Rate *obj;
  double b_expl_temp;
  double expl_temp;
  obj = this;
  obj->RateHelper = MATLABRate_create(20.0);
  MATLABRate_unused(&obj->RateHelper);
  obj->DesiredRate = 20.0;
  tic(&expl_temp, &b_expl_temp);
  tic(&obj->PreviousPeriod.tv_sec, &obj->PreviousPeriod.tv_nsec);
  return obj;
}

} // namespace ros
} // namespace coder

//
// File trailer for Rate.cpp
//
// [EOF]
//
