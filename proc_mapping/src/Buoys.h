//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Buoys.h
//
// Code generation for function 'Buoys'
//

#ifndef BUOYS_H
#define BUOYS_H

// Include files
#include "proc_mapping_internal_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

class b_pointCloud;

namespace vision {
namespace internal {
namespace codegen {
class Kdtree;

}
} // namespace internal
} // namespace vision
} // namespace coder

// Type Definitions
class Buoys {
public:
  void getBuoyPose(coder::pointCloud *subPT, const double auvQuat[4],
                   double p[3], double q[4]) const;
  coder::pointCloud *filteredPT;
  coder::array<unsigned int, 2U> PTlabels;
  coder::b_pointCloud *buoyPT;
  b_struct_T param;
};

// Function Declarations
coder::vision::internal::codegen::Kdtree binary_expand_op(
    coder::pointCloud *in1, const coder::array<unsigned int, 1U> &in2,
    const coder::array<int, 2U> &in3,
    coder::vision::internal::codegen::Kdtree in4, coder::pointCloud *in5);

#endif
// End of code generation (Buoys.h)
