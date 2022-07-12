//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rotationToQuaternion.h
//
// Code generation for function 'rotationToQuaternion'
//

#ifndef ROTATIONTOQUATERNION_H
#define ROTATIONTOQUATERNION_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace vision {
namespace internal {
namespace quaternion {
void rotationToQuaternion(const double R[9], double b_quaternion[4]);

void rotationToQuaternion(const float R[9], float b_quaternion[4]);

} // namespace quaternion
} // namespace internal
} // namespace vision
} // namespace coder

#endif
// End of code generation (rotationToQuaternion.h)
