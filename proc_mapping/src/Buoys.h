//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Buoys.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef BUOYS_H
#define BUOYS_H

// Include Files
#include "proc_mapping_internal_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

class b_pointCloud;

} // namespace coder
struct sonia_common_ObstacleInfoStruct_T;

// Type Definitions
class Buoys {
public:
  void SegementByAtribute(const double auvPose[7],
                          sonia_common_ObstacleInfoStruct_T feature[2]);

private:
  void getBuoyPose(coder::pointCloud *subPT, const double auvQuat[4],
                   double p[3], double q[4], double *confidence) const;

public:
  coder::pointCloud *filteredPT;
  coder::array<unsigned int, 2U> PTlabels;
  coder::b_pointCloud *buoyPT;
  h_struct_T param;
};

#endif
//
// File trailer for Buoys.h
//
// [EOF]
//
