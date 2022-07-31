//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rotationToQuaternion.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "rotationToQuaternion.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const double R[9]
//                double b_quaternion[4]
// Return Type  : void
//
namespace coder {
namespace vision {
namespace internal {
namespace quaternion {
void rotationToQuaternion(const double R[9], double b_quaternion[4])
{
  array<double, 2U> r;
  double b_unnamed_idx_0;
  double c_unnamed_idx_0;
  double d_unnamed_idx_0;
  double maxv;
  double s;
  double t;
  double unnamed_idx_0;
  int trueCount;
  bool b;
  bool b1;
  bool b2;
  bool b3;
  bool b4;
  bool b5;
  t = (R[0] + R[4]) + R[8];
  maxv = std::fmax(R[0], std::fmax(R[4], R[8]));
  b = !(t >= 0.0);
  b1 = (maxv == R[0]);
  b2 = !(t >= 0.0);
  b3 = (maxv == R[4]);
  b4 = !(t >= 0.0);
  b5 = (maxv == R[8]);
  trueCount = 0;
  if (t >= 0.0) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = t + 1.0;
  }
  for (int k{0}; k < trueCount; k++) {
    r[0] = std::sqrt(r[0]);
  }
  maxv = 0.0;
  trueCount = 0;
  if (t >= 0.0) {
    maxv = r[0];
  }
  if (b && b1) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = ((R[0] + 1.0) - R[4]) - R[8];
  }
  for (int k{0}; k < trueCount; k++) {
    r[0] = std::sqrt(r[0]);
  }
  trueCount = 0;
  if (b && b1) {
    maxv = r[0];
  }
  if (b2 && b3) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = ((R[4] + 1.0) - R[0]) - R[8];
  }
  for (int k{0}; k < trueCount; k++) {
    r[0] = std::sqrt(r[0]);
  }
  trueCount = 0;
  if (b2 && b3) {
    maxv = r[0];
  }
  if (b4 && b5) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = ((R[8] + 1.0) - R[0]) - R[4];
  }
  for (int k{0}; k < trueCount; k++) {
    r[0] = std::sqrt(r[0]);
  }
  if (b4 && b5) {
    maxv = r[0];
  }
  s = 0.5 / maxv;
  unnamed_idx_0 = 0.0;
  trueCount = 0;
  if (t >= 0.0) {
    unnamed_idx_0 = 0.5 * maxv;
  }
  if (b && b1) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[5] - R[7]) * s;
  }
  trueCount = 0;
  if (b && b1) {
    unnamed_idx_0 = r[0];
  }
  if (b2 && b3) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[6] - R[2]) * s;
  }
  trueCount = 0;
  if (b2 && b3) {
    unnamed_idx_0 = r[0];
  }
  if (b4 && b5) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[1] - R[3]) * s;
  }
  trueCount = 0;
  if (b4 && b5) {
    unnamed_idx_0 = r[0];
  }
  if (t >= 0.0) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[5] - R[7]) * s;
  }
  b_unnamed_idx_0 = 0.0;
  if (t >= 0.0) {
    b_unnamed_idx_0 = r[0];
  }
  trueCount = 0;
  if (b && b1) {
    b_unnamed_idx_0 = 0.5 * maxv;
  }
  if (b2 && b3) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[1] + R[3]) * s;
  }
  trueCount = 0;
  if (b2 && b3) {
    b_unnamed_idx_0 = r[0];
  }
  if (b4 && b5) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[2] + R[6]) * s;
  }
  trueCount = 0;
  if (b4 && b5) {
    b_unnamed_idx_0 = r[0];
  }
  if (t >= 0.0) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[6] - R[2]) * s;
  }
  c_unnamed_idx_0 = 0.0;
  trueCount = 0;
  if (t >= 0.0) {
    c_unnamed_idx_0 = r[0];
  }
  if (b && b1) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[1] + R[3]) * s;
  }
  if (b && b1) {
    c_unnamed_idx_0 = r[0];
  }
  trueCount = 0;
  if (b2 && b3) {
    c_unnamed_idx_0 = 0.5 * maxv;
  }
  if (b4 && b5) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[5] + R[7]) * s;
  }
  trueCount = 0;
  if (b4 && b5) {
    c_unnamed_idx_0 = r[0];
  }
  if (t >= 0.0) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[1] - R[3]) * s;
  }
  d_unnamed_idx_0 = 0.0;
  trueCount = 0;
  if (t >= 0.0) {
    d_unnamed_idx_0 = r[0];
  }
  if (b && b1) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[2] + R[6]) * s;
  }
  trueCount = 0;
  if (b && b1) {
    d_unnamed_idx_0 = r[0];
  }
  if (b2 && b3) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[5] + R[7]) * s;
  }
  if (b2 && b3) {
    d_unnamed_idx_0 = r[0];
  }
  if (b4 && b5) {
    d_unnamed_idx_0 = 0.5 * maxv;
  }
  b_quaternion[0] = unnamed_idx_0;
  b_quaternion[1] = b_unnamed_idx_0;
  b_quaternion[2] = c_unnamed_idx_0;
  b_quaternion[3] = d_unnamed_idx_0;
}

//
// Arguments    : const float R[9]
//                float b_quaternion[4]
// Return Type  : void
//
void rotationToQuaternion(const float R[9], float b_quaternion[4])
{
  array<float, 2U> r;
  float b_unnamed_idx_0;
  float c_unnamed_idx_0;
  float d_unnamed_idx_0;
  float maxv;
  float s;
  float t;
  float unnamed_idx_0;
  int trueCount;
  bool b;
  bool b1;
  bool b2;
  bool b3;
  bool b4;
  bool b5;
  t = (R[0] + R[4]) + R[8];
  maxv = std::fmax(R[0], std::fmax(R[4], R[8]));
  b = !(t >= 0.0F);
  b1 = (maxv == R[0]);
  b2 = !(t >= 0.0F);
  b3 = (maxv == R[4]);
  b4 = !(t >= 0.0F);
  b5 = (maxv == R[8]);
  trueCount = 0;
  if (t >= 0.0F) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = t + 1.0F;
  }
  for (int k{0}; k < trueCount; k++) {
    r[0] = std::sqrt(r[0]);
  }
  maxv = 0.0F;
  trueCount = 0;
  if (t >= 0.0F) {
    maxv = r[0];
  }
  if (b && b1) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = ((R[0] + 1.0F) - R[4]) - R[8];
  }
  for (int k{0}; k < trueCount; k++) {
    r[0] = std::sqrt(r[0]);
  }
  trueCount = 0;
  if (b && b1) {
    maxv = r[0];
  }
  if (b2 && b3) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = ((R[4] + 1.0F) - R[0]) - R[8];
  }
  for (int k{0}; k < trueCount; k++) {
    r[0] = std::sqrt(r[0]);
  }
  trueCount = 0;
  if (b2 && b3) {
    maxv = r[0];
  }
  if (b4 && b5) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = ((R[8] + 1.0F) - R[0]) - R[4];
  }
  for (int k{0}; k < trueCount; k++) {
    r[0] = std::sqrt(r[0]);
  }
  if (b4 && b5) {
    maxv = r[0];
  }
  s = 0.5F / maxv;
  unnamed_idx_0 = 0.0F;
  trueCount = 0;
  if (t >= 0.0F) {
    unnamed_idx_0 = 0.5F * maxv;
  }
  if (b && b1) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[5] - R[7]) * s;
  }
  trueCount = 0;
  if (b && b1) {
    unnamed_idx_0 = r[0];
  }
  if (b2 && b3) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[6] - R[2]) * s;
  }
  trueCount = 0;
  if (b2 && b3) {
    unnamed_idx_0 = r[0];
  }
  if (b4 && b5) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[1] - R[3]) * s;
  }
  trueCount = 0;
  if (b4 && b5) {
    unnamed_idx_0 = r[0];
  }
  if (t >= 0.0F) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[5] - R[7]) * s;
  }
  b_unnamed_idx_0 = 0.0F;
  if (t >= 0.0F) {
    b_unnamed_idx_0 = r[0];
  }
  trueCount = 0;
  if (b && b1) {
    b_unnamed_idx_0 = 0.5F * maxv;
  }
  if (b2 && b3) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[1] + R[3]) * s;
  }
  trueCount = 0;
  if (b2 && b3) {
    b_unnamed_idx_0 = r[0];
  }
  if (b4 && b5) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[2] + R[6]) * s;
  }
  trueCount = 0;
  if (b4 && b5) {
    b_unnamed_idx_0 = r[0];
  }
  if (t >= 0.0F) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[6] - R[2]) * s;
  }
  c_unnamed_idx_0 = 0.0F;
  trueCount = 0;
  if (t >= 0.0F) {
    c_unnamed_idx_0 = r[0];
  }
  if (b && b1) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[1] + R[3]) * s;
  }
  if (b && b1) {
    c_unnamed_idx_0 = r[0];
  }
  trueCount = 0;
  if (b2 && b3) {
    c_unnamed_idx_0 = 0.5F * maxv;
  }
  if (b4 && b5) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[5] + R[7]) * s;
  }
  trueCount = 0;
  if (b4 && b5) {
    c_unnamed_idx_0 = r[0];
  }
  if (t >= 0.0F) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[1] - R[3]) * s;
  }
  d_unnamed_idx_0 = 0.0F;
  trueCount = 0;
  if (t >= 0.0F) {
    d_unnamed_idx_0 = r[0];
  }
  if (b && b1) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[2] + R[6]) * s;
  }
  trueCount = 0;
  if (b && b1) {
    d_unnamed_idx_0 = r[0];
  }
  if (b2 && b3) {
    trueCount = 1;
  }
  r.set_size(1, trueCount);
  for (int k{0}; k < trueCount; k++) {
    r[0] = (R[5] + R[7]) * s;
  }
  if (b2 && b3) {
    d_unnamed_idx_0 = r[0];
  }
  if (b4 && b5) {
    d_unnamed_idx_0 = 0.5F * maxv;
  }
  b_quaternion[0] = unnamed_idx_0;
  b_quaternion[1] = b_unnamed_idx_0;
  b_quaternion[2] = c_unnamed_idx_0;
  b_quaternion[3] = d_unnamed_idx_0;
}

} // namespace quaternion
} // namespace internal
} // namespace vision
} // namespace coder

//
// File trailer for rotationToQuaternion.cpp
//
// [EOF]
//
