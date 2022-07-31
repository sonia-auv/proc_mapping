//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Rate.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef RATE_H
#define RATE_H

// Include Files
#include "rtwtypes.h"
#include "coder_posix_time.h"
#include "mlroscpp_rate.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class Rate {
public:
  Rate *init();
  std::unique_ptr<MATLABRate> RateHelper;
  double DesiredRate;
  coderTimespec PreviousPeriod;
};

} // namespace ros
} // namespace coder

#endif
//
// File trailer for Rate.h
//
// [EOF]
//
