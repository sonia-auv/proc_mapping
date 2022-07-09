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
  coder::b_pointCloud *buoyPT;

protected:
  coder::pointCloud *filteredPT;

private:
  double clusterDist;
  double planeTol;
  double icpInlierRation;
  double zNormalThres;
  double inPlaneThres;
  double areaThres[2];
  coder::array<unsigned int, 2U> PTlabels;
};

// Function Declarations
coder::vision::internal::codegen::Kdtree binary_expand_op(
    coder::pointCloud *in1, const coder::array<unsigned int, 1U> &in2,
    const coder::array<int, 2U> &in3,
    coder::vision::internal::codegen::Kdtree in4, coder::pointCloud *in5);

#endif
// End of code generation (Buoys.h)
