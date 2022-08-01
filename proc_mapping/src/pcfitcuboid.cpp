//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcfitcuboid.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "pcfitcuboid.h"
#include "Kdtree.h"
#include "cosd.h"
#include "cuboidModel.h"
#include "minOrMax.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "sind.h"
#include "var.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>
#include <string.h>

// Function Declarations
static void b_minus(coder::array<double, 2U> &in1,
                    const coder::array<double, 2U> &in2);

namespace coder {
static void findBoundingBox(const ::coder::array<double, 2U> &X,
                            double boundingBox2d[5], double *theta,
                            double *score);

static void getDistanceMatrix(const ::coder::array<double, 2U> &projections,
                              ::coder::array<double, 2U> &distanceMatrix);

} // namespace coder
static void d_binary_expand_op(coder::array<bool, 1U> &in1,
                               const coder::array<double, 2U> &in2, int in3,
                               const coder::array<double, 2U> &in4);

static void e_binary_expand_op(coder::array<bool, 1U> &in1,
                               const coder::array<double, 2U> &in2, int in3,
                               const coder::array<double, 2U> &in4);

static void lt(coder::array<bool, 2U> &in1, const coder::array<double, 2U> &in2,
               const coder::array<double, 2U> &in3);

static void minus(coder::array<double, 2U> &in1,
                  const coder::array<double, 2U> &in2);

// Function Definitions
//
// Arguments    : coder::array<double, 2U> &in1
//                const coder::array<double, 2U> &in2
// Return Type  : void
//
static void b_minus(coder::array<double, 2U> &in1,
                    const coder::array<double, 2U> &in2)
{
  coder::array<double, 2U> b_in1;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in2.size(0) == 1) {
    i = in1.size(0);
  } else {
    i = in2.size(0);
  }
  b_in1.set_size(i, 91);
  stride_0_0 = (in1.size(0) != 1);
  stride_1_0 = (in2.size(0) != 1);
  if (in2.size(0) == 1) {
    loop_ub = in1.size(0);
  } else {
    loop_ub = in2.size(0);
  }
  for (i = 0; i < 91; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      b_in1[i1 + b_in1.size(0) * i] = in1[i1 * stride_0_0 + in1.size(0) * i] -
                                      in2[i1 * stride_1_0 + in2.size(0) * i];
    }
  }
  in1.set_size(b_in1.size(0), 91);
  loop_ub = b_in1.size(0);
  for (i = 0; i < 91; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      in1[i1 + in1.size(0) * i] = b_in1[i1 + b_in1.size(0) * i];
    }
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &X
//                double boundingBox2d[5]
//                double *theta
//                double *score
// Return Type  : void
//
namespace coder {
static void findBoundingBox(const ::coder::array<double, 2U> &X,
                            double boundingBox2d[5], double *theta,
                            double *score)
{
  static const double dv[182]{1.0,
                              0.99984769515639127,
                              0.99939082701909576,
                              0.99862953475457383,
                              0.9975640502598242,
                              0.99619469809174555,
                              0.99452189536827329,
                              0.992546151641322,
                              0.99026806874157036,
                              0.98768834059513777,
                              0.984807753012208,
                              0.981627183447664,
                              0.97814760073380569,
                              0.97437006478523525,
                              0.97029572627599647,
                              0.96592582628906831,
                              0.96126169593831889,
                              0.95630475596303544,
                              0.95105651629515353,
                              0.94551857559931685,
                              0.93969262078590843,
                              0.93358042649720174,
                              0.92718385456678742,
                              0.92050485345244037,
                              0.91354545764260087,
                              0.90630778703664994,
                              0.898794046299167,
                              0.8910065241883679,
                              0.882947592858927,
                              0.87461970713939574,
                              0.86602540378443871,
                              0.85716730070211233,
                              0.848048096156426,
                              0.838670567945424,
                              0.82903757255504162,
                              0.8191520442889918,
                              0.80901699437494745,
                              0.79863551004729283,
                              0.7880107536067219,
                              0.7771459614569709,
                              0.766044443118978,
                              0.754709580222772,
                              0.74314482547739424,
                              0.73135370161917046,
                              0.71933980033865119,
                              0.70710678118654757,
                              0.69465837045899725,
                              0.68199836006249848,
                              0.66913060635885824,
                              0.65605902899050728,
                              0.64278760968653925,
                              0.62932039104983739,
                              0.61566147532565829,
                              0.60181502315204827,
                              0.58778525229247314,
                              0.573576436351046,
                              0.5591929034707469,
                              0.54463903501502708,
                              0.5299192642332049,
                              0.51503807491005416,
                              0.49999999999999994,
                              0.48480962024633706,
                              0.46947156278589081,
                              0.45399049973954675,
                              0.4383711467890774,
                              0.42261826174069944,
                              0.40673664307580021,
                              0.39073112848927377,
                              0.374606593415912,
                              0.35836794954530027,
                              0.34202014332566871,
                              0.3255681544571567,
                              0.3090169943749474,
                              0.29237170472273677,
                              0.27563735581699916,
                              0.25881904510252074,
                              0.24192189559966773,
                              0.224951054343865,
                              0.20791169081775934,
                              0.1908089953765448,
                              0.17364817766693033,
                              0.15643446504023087,
                              0.13917310096006544,
                              0.12186934340514748,
                              0.10452846326765347,
                              0.087155742747658166,
                              0.0697564737441253,
                              0.052335956242943835,
                              0.034899496702500969,
                              0.017452406437283512,
                              -0.0,
                              0.0,
                              0.017452406437283512,
                              0.034899496702500969,
                              0.052335956242943835,
                              0.0697564737441253,
                              0.087155742747658166,
                              0.10452846326765347,
                              0.12186934340514748,
                              0.13917310096006544,
                              0.15643446504023087,
                              0.17364817766693033,
                              0.1908089953765448,
                              0.20791169081775934,
                              0.224951054343865,
                              0.24192189559966773,
                              0.25881904510252074,
                              0.27563735581699916,
                              0.29237170472273677,
                              0.3090169943749474,
                              0.3255681544571567,
                              0.34202014332566871,
                              0.35836794954530027,
                              0.374606593415912,
                              0.39073112848927377,
                              0.40673664307580021,
                              0.42261826174069944,
                              0.4383711467890774,
                              0.45399049973954675,
                              0.46947156278589081,
                              0.48480962024633706,
                              0.49999999999999994,
                              0.51503807491005416,
                              0.5299192642332049,
                              0.54463903501502708,
                              0.5591929034707469,
                              0.573576436351046,
                              0.58778525229247314,
                              0.60181502315204827,
                              0.61566147532565829,
                              0.62932039104983739,
                              0.64278760968653925,
                              0.65605902899050728,
                              0.66913060635885824,
                              0.68199836006249848,
                              0.69465837045899725,
                              0.70710678118654746,
                              0.71933980033865119,
                              0.73135370161917046,
                              0.74314482547739424,
                              0.754709580222772,
                              0.766044443118978,
                              0.7771459614569709,
                              0.7880107536067219,
                              0.79863551004729283,
                              0.80901699437494745,
                              0.8191520442889918,
                              0.82903757255504162,
                              0.838670567945424,
                              0.848048096156426,
                              0.85716730070211233,
                              0.86602540378443871,
                              0.87461970713939574,
                              0.882947592858927,
                              0.8910065241883679,
                              0.898794046299167,
                              0.90630778703664994,
                              0.91354545764260087,
                              0.92050485345244037,
                              0.92718385456678742,
                              0.93358042649720174,
                              0.93969262078590843,
                              0.94551857559931685,
                              0.95105651629515353,
                              0.95630475596303544,
                              0.96126169593831889,
                              0.96592582628906831,
                              0.97029572627599647,
                              0.97437006478523525,
                              0.97814760073380569,
                              0.981627183447664,
                              0.984807753012208,
                              0.98768834059513777,
                              0.99026806874157036,
                              0.992546151641322,
                              0.99452189536827329,
                              0.99619469809174555,
                              0.9975640502598242,
                              0.99862953475457383,
                              0.99939082701909576,
                              0.99984769515639127,
                              1.0};
  static const double dv1[182]{-0.0,
                               -0.017452406437283512,
                               -0.034899496702500969,
                               -0.052335956242943835,
                               -0.0697564737441253,
                               -0.087155742747658166,
                               -0.10452846326765347,
                               -0.12186934340514748,
                               -0.13917310096006544,
                               -0.15643446504023087,
                               -0.17364817766693033,
                               -0.1908089953765448,
                               -0.20791169081775934,
                               -0.224951054343865,
                               -0.24192189559966773,
                               -0.25881904510252074,
                               -0.27563735581699916,
                               -0.29237170472273677,
                               -0.3090169943749474,
                               -0.3255681544571567,
                               -0.34202014332566871,
                               -0.35836794954530027,
                               -0.374606593415912,
                               -0.39073112848927377,
                               -0.40673664307580021,
                               -0.42261826174069944,
                               -0.4383711467890774,
                               -0.45399049973954675,
                               -0.46947156278589081,
                               -0.48480962024633706,
                               -0.49999999999999994,
                               -0.51503807491005416,
                               -0.5299192642332049,
                               -0.54463903501502708,
                               -0.5591929034707469,
                               -0.573576436351046,
                               -0.58778525229247314,
                               -0.60181502315204827,
                               -0.61566147532565829,
                               -0.62932039104983739,
                               -0.64278760968653925,
                               -0.65605902899050728,
                               -0.66913060635885824,
                               -0.68199836006249848,
                               -0.69465837045899725,
                               -0.70710678118654746,
                               -0.71933980033865119,
                               -0.73135370161917046,
                               -0.74314482547739424,
                               -0.754709580222772,
                               -0.766044443118978,
                               -0.7771459614569709,
                               -0.7880107536067219,
                               -0.79863551004729283,
                               -0.80901699437494745,
                               -0.8191520442889918,
                               -0.82903757255504162,
                               -0.838670567945424,
                               -0.848048096156426,
                               -0.85716730070211233,
                               -0.86602540378443871,
                               -0.87461970713939574,
                               -0.882947592858927,
                               -0.8910065241883679,
                               -0.898794046299167,
                               -0.90630778703664994,
                               -0.91354545764260087,
                               -0.92050485345244037,
                               -0.92718385456678742,
                               -0.93358042649720174,
                               -0.93969262078590843,
                               -0.94551857559931685,
                               -0.95105651629515353,
                               -0.95630475596303544,
                               -0.96126169593831889,
                               -0.96592582628906831,
                               -0.97029572627599647,
                               -0.97437006478523525,
                               -0.97814760073380569,
                               -0.981627183447664,
                               -0.984807753012208,
                               -0.98768834059513777,
                               -0.99026806874157036,
                               -0.992546151641322,
                               -0.99452189536827329,
                               -0.99619469809174555,
                               -0.9975640502598242,
                               -0.99862953475457383,
                               -0.99939082701909576,
                               -0.99984769515639127,
                               -1.0,
                               1.0,
                               0.99984769515639127,
                               0.99939082701909576,
                               0.99862953475457383,
                               0.9975640502598242,
                               0.99619469809174555,
                               0.99452189536827329,
                               0.992546151641322,
                               0.99026806874157036,
                               0.98768834059513777,
                               0.984807753012208,
                               0.981627183447664,
                               0.97814760073380569,
                               0.97437006478523525,
                               0.97029572627599647,
                               0.96592582628906831,
                               0.96126169593831889,
                               0.95630475596303544,
                               0.95105651629515353,
                               0.94551857559931685,
                               0.93969262078590843,
                               0.93358042649720174,
                               0.92718385456678742,
                               0.92050485345244037,
                               0.91354545764260087,
                               0.90630778703664994,
                               0.898794046299167,
                               0.8910065241883679,
                               0.882947592858927,
                               0.87461970713939574,
                               0.86602540378443871,
                               0.85716730070211233,
                               0.848048096156426,
                               0.838670567945424,
                               0.82903757255504162,
                               0.8191520442889918,
                               0.80901699437494745,
                               0.79863551004729283,
                               0.7880107536067219,
                               0.7771459614569709,
                               0.766044443118978,
                               0.754709580222772,
                               0.74314482547739424,
                               0.73135370161917046,
                               0.71933980033865119,
                               0.70710678118654757,
                               0.69465837045899725,
                               0.68199836006249848,
                               0.66913060635885824,
                               0.65605902899050728,
                               0.64278760968653925,
                               0.62932039104983739,
                               0.61566147532565829,
                               0.60181502315204827,
                               0.58778525229247314,
                               0.573576436351046,
                               0.5591929034707469,
                               0.54463903501502708,
                               0.5299192642332049,
                               0.51503807491005416,
                               0.49999999999999994,
                               0.48480962024633706,
                               0.46947156278589081,
                               0.45399049973954675,
                               0.4383711467890774,
                               0.42261826174069944,
                               0.40673664307580021,
                               0.39073112848927377,
                               0.374606593415912,
                               0.35836794954530027,
                               0.34202014332566871,
                               0.3255681544571567,
                               0.3090169943749474,
                               0.29237170472273677,
                               0.27563735581699916,
                               0.25881904510252074,
                               0.24192189559966773,
                               0.224951054343865,
                               0.20791169081775934,
                               0.1908089953765448,
                               0.17364817766693033,
                               0.15643446504023087,
                               0.13917310096006544,
                               0.12186934340514748,
                               0.10452846326765347,
                               0.087155742747658166,
                               0.0697564737441253,
                               0.052335956242943835,
                               0.034899496702500969,
                               0.017452406437283512,
                               -0.0};
  array<double, 2U> distance1;
  array<double, 2U> projection1;
  array<double, 2U> projection2;
  array<double, 1U> C1key;
  array<double, 1U> C2key;
  array<int, 1U> r1;
  array<int, 1U> r2;
  array<int, 1U> r3;
  array<int, 1U> r4;
  array<bool, 1U> r;
  double scores[91];
  double A_tmp;
  double B_idx_0;
  double b_distance1;
  double d;
  double maxScore;
  double vertices_idx_0;
  double vertices_idx_1;
  double vertices_idx_2;
  double vertices_idx_4;
  double vertices_idx_5;
  double vertices_idx_6;
  double yaw;
  int coffset;
  int idx;
  int j;
  int m;
  bool b;
  bool exitg1;
  m = X.size(0);
  projection1.set_size(X.size(0), 91);
  idx = X.size(0);
  projection2.set_size(X.size(0), 91);
  for (j = 0; j < 91; j++) {
    coffset = j * m;
    for (int i{0}; i < m; i++) {
      projection1[coffset + i] = X[i] * dv[j] + X[X.size(0) + i] * dv[j + 91];
    }
    coffset = j * idx;
    for (int i{0}; i < idx; i++) {
      projection2[coffset + i] = X[i] * dv1[j] + X[X.size(0) + i] * dv1[j + 91];
    }
    scores[j] = 0.0;
  }
  getDistanceMatrix(projection1, distance1);
  getDistanceMatrix(projection2, projection1);
  for (int i{0}; i < 91; i++) {
    idx = distance1.size(0);
    if (distance1.size(0) == projection1.size(0)) {
      r.set_size(distance1.size(0));
      for (m = 0; m < idx; m++) {
        r[m] = (distance1[m + distance1.size(0) * i] <=
                projection1[m + projection1.size(0) * i]);
      }
    } else {
      e_binary_expand_op(r, distance1, i, projection1);
    }
    coffset = r.size(0) - 1;
    idx = 0;
    for (m = 0; m <= coffset; m++) {
      if (r[m]) {
        idx++;
      }
    }
    r1.set_size(idx);
    idx = 0;
    for (m = 0; m <= coffset; m++) {
      if (r[m]) {
        r1[idx] = m + 1;
        idx++;
      }
    }
    if (r1.size(0) > 1) {
      idx = distance1.size(0);
      if (distance1.size(0) == projection1.size(0)) {
        r.set_size(distance1.size(0));
        for (m = 0; m < idx; m++) {
          r[m] = (distance1[m + distance1.size(0) * i] <=
                  projection1[m + projection1.size(0) * i]);
        }
      } else {
        e_binary_expand_op(r, distance1, i, projection1);
      }
      coffset = r.size(0) - 1;
      idx = 0;
      for (m = 0; m <= coffset; m++) {
        if (r[m]) {
          idx++;
        }
      }
      r2.set_size(idx);
      idx = 0;
      for (m = 0; m <= coffset; m++) {
        if (r[m]) {
          r2[idx] = m + 1;
          idx++;
        }
      }
      idx = r2.size(0);
      C1key.set_size(r2.size(0));
      for (m = 0; m < idx; m++) {
        C1key[m] = distance1[(r2[m] + distance1.size(0) * i) - 1];
      }
      scores[i] += var(C1key);
    }
    idx = projection1.size(0);
    if (projection1.size(0) == distance1.size(0)) {
      r.set_size(projection1.size(0));
      for (m = 0; m < idx; m++) {
        r[m] = (projection1[m + projection1.size(0) * i] <
                distance1[m + distance1.size(0) * i]);
      }
    } else {
      d_binary_expand_op(r, projection1, i, distance1);
    }
    coffset = r.size(0) - 1;
    idx = 0;
    for (m = 0; m <= coffset; m++) {
      if (r[m]) {
        idx++;
      }
    }
    r3.set_size(idx);
    idx = 0;
    for (m = 0; m <= coffset; m++) {
      if (r[m]) {
        r3[idx] = m + 1;
        idx++;
      }
    }
    if (r3.size(0) > 1) {
      idx = projection1.size(0);
      if (projection1.size(0) == distance1.size(0)) {
        r.set_size(projection1.size(0));
        for (m = 0; m < idx; m++) {
          r[m] = (projection1[m + projection1.size(0) * i] <
                  distance1[m + distance1.size(0) * i]);
        }
      } else {
        d_binary_expand_op(r, projection1, i, distance1);
      }
      coffset = r.size(0) - 1;
      idx = 0;
      for (m = 0; m <= coffset; m++) {
        if (r[m]) {
          idx++;
        }
      }
      r4.set_size(idx);
      idx = 0;
      for (m = 0; m <= coffset; m++) {
        if (r[m]) {
          r4[idx] = m + 1;
          idx++;
        }
      }
      idx = r4.size(0);
      C1key.set_size(r4.size(0));
      for (m = 0; m < idx; m++) {
        C1key[m] = projection1[(r4[m] + projection1.size(0) * i) - 1];
      }
      scores[i] += var(C1key);
    }
  }
  b = std::isnan(scores[0]);
  if (!b) {
    j = 1;
  } else {
    j = 0;
    coffset = 2;
    exitg1 = false;
    while ((!exitg1) && (coffset < 92)) {
      if (!std::isnan(scores[coffset - 1])) {
        j = coffset;
        exitg1 = true;
      } else {
        coffset++;
      }
    }
  }
  if (j == 0) {
    yaw = scores[0];
    j = 1;
  } else {
    yaw = scores[j - 1];
    m = j + 1;
    for (coffset = m; coffset < 92; coffset++) {
      d = scores[coffset - 1];
      if (yaw > d) {
        yaw = d;
        j = coffset;
      }
    }
  }
  if (!b) {
    idx = 1;
  } else {
    idx = 0;
    coffset = 2;
    exitg1 = false;
    while ((!exitg1) && (coffset <= 91)) {
      if (!std::isnan(scores[coffset - 1])) {
        idx = coffset;
        exitg1 = true;
      } else {
        coffset++;
      }
    }
  }
  if (idx == 0) {
    maxScore = scores[0];
  } else {
    maxScore = scores[idx - 1];
    m = idx + 1;
    for (coffset = m; coffset < 92; coffset++) {
      d = scores[coffset - 1];
      if (maxScore < d) {
        maxScore = d;
      }
    }
  }
  *score = 1.0 - yaw / maxScore;
  *theta = static_cast<double>(j) - 1.0;
  yaw = static_cast<double>(j) - 1.0;
  b_sind(&yaw);
  B_idx_0 = static_cast<double>(j) - 1.0;
  b_cosd(&B_idx_0);
  m = X.size(0);
  C1key.set_size(X.size(0));
  for (int i{0}; i < m; i++) {
    C1key[i] = X[i] * B_idx_0 + X[X.size(0) + i] * yaw;
  }
  m = X.size(0);
  C2key.set_size(X.size(0));
  for (int i{0}; i < m; i++) {
    C2key[i] = X[i] * -yaw + X[X.size(0) + i] * B_idx_0;
  }
  b_distance1 = static_cast<double>(j) - 1.0;
  b_sind(&b_distance1);
  A_tmp = static_cast<double>(j) - 1.0;
  b_cosd(&A_tmp);
  d = internal::minimum(C1key);
  maxScore = internal::minimum(C2key);
  yaw = internal::maximum(C1key);
  vertices_idx_2 = internal::maximum(C2key);
  if (A_tmp == 0.0) {
    B_idx_0 = maxScore / -b_distance1;
  } else if (b_distance1 == 0.0) {
    B_idx_0 = yaw / A_tmp;
  } else {
    B_idx_0 = (yaw / b_distance1 - maxScore / A_tmp) /
              (A_tmp / b_distance1 - -b_distance1 / A_tmp);
  }
  if (A_tmp != 0.0) {
    yaw = b_distance1 * B_idx_0 / A_tmp + maxScore / A_tmp;
  } else {
    yaw = -A_tmp * B_idx_0 / b_distance1 + yaw / b_distance1;
  }
  vertices_idx_0 = B_idx_0;
  vertices_idx_4 = yaw;
  if (b_distance1 == 0.0) {
    B_idx_0 = d / A_tmp;
  } else if (A_tmp == 0.0) {
    B_idx_0 = maxScore / -b_distance1;
  } else {
    B_idx_0 = (maxScore / A_tmp - d / b_distance1) /
              (-b_distance1 / A_tmp - A_tmp / b_distance1);
  }
  if (b_distance1 != 0.0) {
    yaw = -A_tmp * B_idx_0 / b_distance1 + d / b_distance1;
  } else {
    yaw = b_distance1 * B_idx_0 / A_tmp + maxScore / A_tmp;
  }
  vertices_idx_1 = B_idx_0;
  vertices_idx_5 = yaw;
  if (b_distance1 == 0.0) {
    B_idx_0 = d / A_tmp;
  } else if (A_tmp == 0.0) {
    B_idx_0 = vertices_idx_2 / -b_distance1;
  } else {
    B_idx_0 = (vertices_idx_2 / A_tmp - d / b_distance1) /
              (-b_distance1 / A_tmp - A_tmp / b_distance1);
  }
  if (b_distance1 != 0.0) {
    yaw = -A_tmp * B_idx_0 / b_distance1 + d / b_distance1;
  } else {
    yaw = b_distance1 * B_idx_0 / A_tmp + vertices_idx_2 / A_tmp;
  }
  vertices_idx_2 = B_idx_0;
  d = vertices_idx_0 - vertices_idx_1;
  maxScore = d * d;
  B_idx_0 -= vertices_idx_1;
  vertices_idx_6 = yaw;
  d = vertices_idx_4 - vertices_idx_5;
  yaw -= vertices_idx_5;
  b_distance1 = std::sqrt(maxScore + d * d);
  maxScore = std::sqrt(B_idx_0 * B_idx_0 + yaw * yaw);
  if (b_distance1 > maxScore) {
    B_idx_0 = vertices_idx_0;
    yaw = vertices_idx_4;
    A_tmp = b_distance1;
    b_distance1 = maxScore;
  } else {
    B_idx_0 = vertices_idx_2;
    yaw = vertices_idx_6;
    A_tmp = maxScore;
  }
  d = vertices_idx_5 - yaw;
  if (d != 0.0) {
    yaw = 57.295779513082323 * std::atan(d / (vertices_idx_1 - B_idx_0));
  } else {
    yaw = 0.0;
  }
  boundingBox2d[0] =
      (vertices_idx_1 + ((vertices_idx_0 + vertices_idx_2) - vertices_idx_1)) /
      2.0;
  boundingBox2d[1] =
      (vertices_idx_5 + ((vertices_idx_4 + vertices_idx_6) - vertices_idx_5)) /
      2.0;
  boundingBox2d[2] = A_tmp;
  boundingBox2d[3] = b_distance1;
  boundingBox2d[4] = yaw;
}

//
// Arguments    : const ::coder::array<double, 2U> &projections
//                ::coder::array<double, 2U> &distanceMatrix
// Return Type  : void
//
static void getDistanceMatrix(const ::coder::array<double, 2U> &projections,
                              ::coder::array<double, 2U> &distanceMatrix)
{
  array<double, 2U> toMax;
  array<double, 2U> x;
  array<int, 1U> r;
  array<bool, 2U> b_index;
  double Cmax[91];
  double Cmin[91];
  int b_m;
  int ibmat;
  int m;
  int ntilerows;
  m = projections.size(0);
  b_m = projections.size(0);
  x.set_size(projections.size(0), 91);
  ntilerows = projections.size(0);
  for (int j{0}; j < 91; j++) {
    double b;
    bool p;
    Cmax[j] = projections[projections.size(0) * j];
    for (ibmat = 2; ibmat <= m; ibmat++) {
      b = projections[(ibmat + projections.size(0) * j) - 1];
      if (std::isnan(b)) {
        p = false;
      } else if (std::isnan(Cmax[j])) {
        p = true;
      } else {
        p = (Cmax[j] < b);
      }
      if (p) {
        Cmax[j] = b;
      }
    }
    Cmin[j] = projections[projections.size(0) * j];
    for (ibmat = 2; ibmat <= b_m; ibmat++) {
      b = projections[(ibmat + projections.size(0) * j) - 1];
      if (std::isnan(b)) {
        p = false;
      } else if (std::isnan(Cmin[j])) {
        p = true;
      } else {
        p = (Cmin[j] > b);
      }
      if (p) {
        Cmin[j] = b;
      }
    }
    ibmat = j * ntilerows;
    for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
      x[ibmat + itilerow] = Cmax[j];
    }
  }
  if (x.size(0) == projections.size(0)) {
    m = x.size(0) * 91;
    x.set_size(x.size(0), 91);
    for (b_m = 0; b_m < m; b_m++) {
      x[b_m] = x[b_m] - projections[b_m];
    }
  } else {
    b_minus(x, projections);
  }
  m = x.size(0) * 91;
  toMax.set_size(x.size(0), 91);
  for (b_m = 0; b_m < m; b_m++) {
    toMax[b_m] = std::abs(x[b_m]);
  }
  x.set_size(projections.size(0), 91);
  ntilerows = projections.size(0);
  for (m = 0; m < 91; m++) {
    ibmat = m * ntilerows;
    for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
      x[ibmat + itilerow] = Cmin[m];
    }
  }
  if (projections.size(0) == x.size(0)) {
    m = projections.size(0) * 91;
    x.set_size(projections.size(0), 91);
    for (b_m = 0; b_m < m; b_m++) {
      x[b_m] = projections[b_m] - x[b_m];
    }
  } else {
    minus(x, projections);
  }
  m = x.size(0) * 91;
  distanceMatrix.set_size(x.size(0), 91);
  for (b_m = 0; b_m < m; b_m++) {
    distanceMatrix[b_m] = std::abs(x[b_m]);
  }
  if (toMax.size(0) == distanceMatrix.size(0)) {
    b_index.set_size(toMax.size(0), 91);
    m = toMax.size(0) * 91;
    for (b_m = 0; b_m < m; b_m++) {
      b_index[b_m] = (toMax[b_m] < distanceMatrix[b_m]);
    }
  } else {
    lt(b_index, toMax, distanceMatrix);
  }
  b_m = b_index.size(0) * 91 - 1;
  m = 0;
  for (ibmat = 0; ibmat <= b_m; ibmat++) {
    if (b_index[ibmat]) {
      m++;
    }
  }
  r.set_size(m);
  m = 0;
  for (ibmat = 0; ibmat <= b_m; ibmat++) {
    if (b_index[ibmat]) {
      r[m] = ibmat + 1;
      m++;
    }
  }
  m = r.size(0);
  for (b_m = 0; b_m < m; b_m++) {
    distanceMatrix[r[b_m] - 1] = toMax[r[b_m] - 1];
  }
}

//
// Arguments    : coder::array<bool, 1U> &in1
//                const coder::array<double, 2U> &in2
//                int in3
//                const coder::array<double, 2U> &in4
// Return Type  : void
//
} // namespace coder
static void d_binary_expand_op(coder::array<bool, 1U> &in1,
                               const coder::array<double, 2U> &in2, int in3,
                               const coder::array<double, 2U> &in4)
{
  int b_loop_ub;
  int i;
  int loop_ub;
  int stride_1_0;
  i = in2.size(0);
  loop_ub = in4.size(0);
  if (loop_ub == 1) {
    b_loop_ub = i;
  } else {
    b_loop_ub = loop_ub;
  }
  in1.set_size(b_loop_ub);
  b_loop_ub = (i != 1);
  stride_1_0 = (loop_ub != 1);
  if (loop_ub == 1) {
    loop_ub = i;
  }
  for (i = 0; i < loop_ub; i++) {
    in1[i] = (in2[i * b_loop_ub + in2.size(0) * in3] <
              in4[i * stride_1_0 + in4.size(0) * in3]);
  }
}

//
// Arguments    : coder::array<bool, 1U> &in1
//                const coder::array<double, 2U> &in2
//                int in3
//                const coder::array<double, 2U> &in4
// Return Type  : void
//
static void e_binary_expand_op(coder::array<bool, 1U> &in1,
                               const coder::array<double, 2U> &in2, int in3,
                               const coder::array<double, 2U> &in4)
{
  int b_loop_ub;
  int i;
  int loop_ub;
  int stride_1_0;
  i = in2.size(0);
  loop_ub = in4.size(0);
  if (loop_ub == 1) {
    b_loop_ub = i;
  } else {
    b_loop_ub = loop_ub;
  }
  in1.set_size(b_loop_ub);
  b_loop_ub = (i != 1);
  stride_1_0 = (loop_ub != 1);
  if (loop_ub == 1) {
    loop_ub = i;
  }
  for (i = 0; i < loop_ub; i++) {
    in1[i] = (in2[i * b_loop_ub + in2.size(0) * in3] <=
              in4[i * stride_1_0 + in4.size(0) * in3]);
  }
}

//
// Arguments    : coder::array<bool, 2U> &in1
//                const coder::array<double, 2U> &in2
//                const coder::array<double, 2U> &in3
// Return Type  : void
//
static void lt(coder::array<bool, 2U> &in1, const coder::array<double, 2U> &in2,
               const coder::array<double, 2U> &in3)
{
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in3.size(0) == 1) {
    i = in2.size(0);
  } else {
    i = in3.size(0);
  }
  in1.set_size(i, 91);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in3.size(0) != 1);
  if (in3.size(0) == 1) {
    loop_ub = in2.size(0);
  } else {
    loop_ub = in3.size(0);
  }
  for (i = 0; i < 91; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      in1[i1 + in1.size(0) * i] = (in2[i1 * stride_0_0 + in2.size(0) * i] <
                                   in3[i1 * stride_1_0 + in3.size(0) * i]);
    }
  }
}

//
// Arguments    : coder::array<double, 2U> &in1
//                const coder::array<double, 2U> &in2
// Return Type  : void
//
static void minus(coder::array<double, 2U> &in1,
                  const coder::array<double, 2U> &in2)
{
  coder::array<double, 2U> b_in2;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in1.size(0) == 1) {
    i = in2.size(0);
  } else {
    i = in1.size(0);
  }
  b_in2.set_size(i, 91);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in1.size(0) != 1);
  if (in1.size(0) == 1) {
    loop_ub = in2.size(0);
  } else {
    loop_ub = in1.size(0);
  }
  for (i = 0; i < 91; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      b_in2[i1 + b_in2.size(0) * i] = in2[i1 * stride_0_0 + in2.size(0) * i] -
                                      in1[i1 * stride_1_0 + in1.size(0) * i];
    }
  }
  in1.set_size(b_in2.size(0), 91);
  loop_ub = b_in2.size(0);
  for (i = 0; i < 91; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      in1[i1 + in1.size(0) * i] = b_in2[i1 + b_in2.size(0) * i];
    }
  }
}

//
// Arguments    : const pointCloud *ptCloudIn
//                const ::coder::array<double, 1U> &varargin_1
//                cuboidModel *iobj_0
// Return Type  : cuboidModel *
//
namespace coder {
cuboidModel *pcfitcuboid(const pointCloud *ptCloudIn,
                         const ::coder::array<double, 1U> &varargin_1,
                         cuboidModel *iobj_0)
{
  c_pointCloud lobj_2[2];
  c_pointCloud ptCLoudValid;
  c_pointCloud *iobj_1;
  c_pointCloud *ptCloud;
  cuboidModel *model;
  pointclouds::internal::codegen::pc::pointCloudArray r;
  vision::internal::codegen::Kdtree lobj_1[3];
  vision::internal::codegen::Kdtree *b_iobj_0;
  array<double, 2U> c_loc;
  array<double, 2U> intensity;
  array<double, 2U> loc;
  array<double, 2U> location;
  array<double, 2U> normal;
  array<double, 2U> nv;
  array<double, 2U> nv_tmp;
  array<double, 2U> rangeData;
  array<double, 1U> b_loc;
  array<int, 1U> r2;
  array<int, 1U> r3;
  array<int, 1U> r4;
  array<int, 1U> r5;
  array<int, 1U> r6;
  array<int, 1U> y;
  array<unsigned char, 2U> C_tmp;
  array<unsigned char, 2U> color;
  array<bool, 2U> r1;
  array<bool, 2U> x;
  array<bool, 1U> indices;
  double boundingBox[9];
  double a__1;
  double a__2;
  int vstride;
  int xoffset;
  lobj_2[0].matlabCodegenIsDeleted = true;
  lobj_2[1].matlabCodegenIsDeleted = true;
  ptCloud = &lobj_2[1];
  b_iobj_0 = &lobj_1[0];
  C_tmp.set_size(0, 0);
  nv_tmp.set_size(0, 0);
  ptCloud->Location.set_size(div_nzp_s32(ptCloudIn->Location.size(0) * 3, 3),
                             3);
  vstride = div_nzp_s32(ptCloudIn->Location.size(0) * 3, 3) * 3;
  for (int j{0}; j < vstride; j++) {
    ptCloud->Location[j] = ptCloudIn->Location[j];
  }
  ptCloud->Color.set_size(0, 0);
  ptCloud->Normal.set_size(0, 0);
  ptCloud->Intensity.set_size(0, 0);
  ptCloud->RangeData.set_size(0, 0);
  ptCloud->PointCloudArrayData.set_size(1, 1);
  ptCloud->PointCloudArrayData[0] = r;
  ptCloud->Kdtree = b_iobj_0;
  ptCloud->matlabCodegenIsDeleted = false;
  if (varargin_1.size(0) > 0) {
    b_iobj_0 = &lobj_1[1];
    iobj_1 = &lobj_2[0];
    location.set_size(ptCloud->Location.size(0), 3);
    vstride = ptCloud->Location.size(0) * 3;
    for (int j{0}; j < vstride; j++) {
      location[j] = ptCloud->Location[j];
    }
    color.set_size(ptCloud->Color.size(0), ptCloud->Color.size(1));
    vstride = ptCloud->Color.size(0) * ptCloud->Color.size(1);
    for (int j{0}; j < vstride; j++) {
      color[j] = ptCloud->Color[j];
    }
    normal.set_size(ptCloud->Normal.size(0), ptCloud->Normal.size(1));
    vstride = ptCloud->Normal.size(0) * ptCloud->Normal.size(1);
    for (int j{0}; j < vstride; j++) {
      normal[j] = ptCloud->Normal[j];
    }
    intensity.set_size(ptCloud->Intensity.size(0), ptCloud->Intensity.size(1));
    vstride = ptCloud->Intensity.size(0) * ptCloud->Intensity.size(1);
    for (int j{0}; j < vstride; j++) {
      intensity[j] = ptCloud->Intensity[j];
    }
    rangeData.set_size(ptCloud->RangeData.size(0), ptCloud->RangeData.size(1));
    vstride = ptCloud->RangeData.size(0) * ptCloud->RangeData.size(1);
    for (int j{0}; j < vstride; j++) {
      rangeData[j] = ptCloud->RangeData[j];
    }
    if (location.size(0) != 0) {
      loc.set_size(varargin_1.size(0), 3);
      vstride = varargin_1.size(0);
      for (int j{0}; j < 3; j++) {
        for (int k{0}; k < vstride; k++) {
          loc[k + loc.size(0) * j] = location[(static_cast<int>(varargin_1[k]) +
                                               location.size(0) * j) -
                                              1];
        }
      }
    } else {
      loc.set_size(0, 3);
    }
    if ((color.size(0) != 0) && (color.size(1) != 0)) {
      vstride = color.size(1);
      C_tmp.set_size(varargin_1.size(0), color.size(1));
      xoffset = varargin_1.size(0);
      for (int j{0}; j < vstride; j++) {
        for (int k{0}; k < xoffset; k++) {
          C_tmp[k + C_tmp.size(0) * j] =
              color[(static_cast<int>(varargin_1[k]) + color.size(0) * j) - 1];
        }
      }
    }
    if ((normal.size(0) != 0) && (normal.size(1) != 0)) {
      vstride = normal.size(1);
      nv.set_size(varargin_1.size(0), normal.size(1));
      xoffset = varargin_1.size(0);
      for (int j{0}; j < vstride; j++) {
        for (int k{0}; k < xoffset; k++) {
          nv[k + nv.size(0) * j] =
              normal[(static_cast<int>(varargin_1[k]) + normal.size(0) * j) -
                     1];
        }
      }
    } else {
      nv.set_size(0, 0);
    }
    if ((intensity.size(0) != 0) && (intensity.size(1) != 0)) {
      vstride = intensity.size(1);
      normal.set_size(varargin_1.size(0), intensity.size(1));
      xoffset = varargin_1.size(0);
      for (int j{0}; j < vstride; j++) {
        for (int k{0}; k < xoffset; k++) {
          normal[k + normal.size(0) * j] = intensity
              [(static_cast<int>(varargin_1[k]) + intensity.size(0) * j) - 1];
        }
      }
    } else {
      normal.set_size(0, 0);
    }
    if ((rangeData.size(0) != 0) && (rangeData.size(1) != 0)) {
      vstride = rangeData.size(1);
      nv_tmp.set_size(varargin_1.size(0), rangeData.size(1));
      xoffset = varargin_1.size(0);
      for (int j{0}; j < vstride; j++) {
        for (int k{0}; k < xoffset; k++) {
          nv_tmp[k + nv_tmp.size(0) * j] = rangeData
              [(static_cast<int>(varargin_1[k]) + rangeData.size(0) * j) - 1];
        }
      }
    }
    ptCloud = iobj_1;
    iobj_1->Location.set_size(loc.size(0), 3);
    vstride = loc.size(0) * 3;
    for (int j{0}; j < vstride; j++) {
      iobj_1->Location[j] = loc[j];
    }
    iobj_1->Color.set_size(C_tmp.size(0), C_tmp.size(1));
    vstride = C_tmp.size(0) * C_tmp.size(1);
    for (int j{0}; j < vstride; j++) {
      iobj_1->Color[j] = C_tmp[j];
    }
    iobj_1->Normal.set_size(nv.size(0), nv.size(1));
    vstride = nv.size(0) * nv.size(1);
    for (int j{0}; j < vstride; j++) {
      iobj_1->Normal[j] = nv[j];
    }
    iobj_1->Intensity.set_size(normal.size(0), normal.size(1));
    vstride = normal.size(0) * normal.size(1);
    for (int j{0}; j < vstride; j++) {
      iobj_1->Intensity[j] = normal[j];
    }
    iobj_1->RangeData.set_size(0, 0);
    iobj_1->PointCloudArrayData.set_size(1, 1);
    iobj_1->PointCloudArrayData[0] = r;
    iobj_1->Kdtree = b_iobj_0;
    iobj_1->matlabCodegenIsDeleted = false;
    iobj_1->RangeData.set_size(nv_tmp.size(0), nv_tmp.size(1));
    vstride = nv_tmp.size(0) * nv_tmp.size(1);
    for (int j{0}; j < vstride; j++) {
      iobj_1->RangeData[j] = nv_tmp[j];
    }
  }
  b_iobj_0 = &lobj_1[2];
  location.set_size(ptCloud->Location.size(0), 3);
  vstride = ptCloud->Location.size(0) * 3;
  for (int j{0}; j < vstride; j++) {
    location[j] = ptCloud->Location[j];
  }
  x.set_size(location.size(0), 3);
  vstride = location.size(0) * 3;
  for (int j{0}; j < vstride; j++) {
    x[j] = std::isinf(location[j]);
  }
  r1.set_size(location.size(0), 3);
  vstride = location.size(0) * 3;
  for (int j{0}; j < vstride; j++) {
    r1[j] = std::isnan(location[j]);
  }
  vstride = x.size(0) * 3;
  x.set_size(x.size(0), 3);
  for (int j{0}; j < vstride; j++) {
    x[j] = ((!x[j]) && (!r1[j]));
  }
  if (x.size(0) == 0) {
    y.set_size(0);
  } else {
    vstride = x.size(0);
    y.set_size(x.size(0));
    for (int j{0}; j < vstride; j++) {
      y[j] = x[j];
    }
    for (int k{0}; k < 2; k++) {
      xoffset = (k + 1) * vstride;
      for (int j{0}; j < vstride; j++) {
        y[j] = y[j] + x[xoffset + j];
      }
    }
  }
  indices.set_size(y.size(0));
  vstride = y.size(0);
  for (int j{0}; j < vstride; j++) {
    indices[j] = (y[j] == 3);
  }
  location.set_size(ptCloud->Location.size(0), 3);
  vstride = ptCloud->Location.size(0) * 3;
  for (int j{0}; j < vstride; j++) {
    location[j] = ptCloud->Location[j];
  }
  color.set_size(ptCloud->Color.size(0), ptCloud->Color.size(1));
  vstride = ptCloud->Color.size(0) * ptCloud->Color.size(1);
  for (int j{0}; j < vstride; j++) {
    color[j] = ptCloud->Color[j];
  }
  normal.set_size(ptCloud->Normal.size(0), ptCloud->Normal.size(1));
  vstride = ptCloud->Normal.size(0) * ptCloud->Normal.size(1);
  for (int j{0}; j < vstride; j++) {
    normal[j] = ptCloud->Normal[j];
  }
  intensity.set_size(ptCloud->Intensity.size(0), ptCloud->Intensity.size(1));
  vstride = ptCloud->Intensity.size(0) * ptCloud->Intensity.size(1);
  for (int j{0}; j < vstride; j++) {
    intensity[j] = ptCloud->Intensity[j];
  }
  rangeData.set_size(ptCloud->RangeData.size(0), ptCloud->RangeData.size(1));
  vstride = ptCloud->RangeData.size(0) * ptCloud->RangeData.size(1);
  for (int j{0}; j < vstride; j++) {
    rangeData[j] = ptCloud->RangeData[j];
  }
  if (location.size(0) != 0) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r2.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r2[vstride] = j + 1;
        vstride++;
      }
    }
    loc.set_size(r2.size(0), 3);
    vstride = r2.size(0);
    for (int j{0}; j < 3; j++) {
      for (int k{0}; k < vstride; k++) {
        loc[k + loc.size(0) * j] = location[(r2[k] + location.size(0) * j) - 1];
      }
    }
  } else {
    loc.set_size(0, 3);
  }
  if ((color.size(0) != 0) && (color.size(1) != 0)) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r3.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r3[vstride] = j + 1;
        vstride++;
      }
    }
    vstride = color.size(1);
    ptCLoudValid.Color.set_size(r3.size(0), color.size(1));
    xoffset = r3.size(0);
    for (int j{0}; j < vstride; j++) {
      for (int k{0}; k < xoffset; k++) {
        ptCLoudValid.Color[k + ptCLoudValid.Color.size(0) * j] =
            color[(r3[k] + color.size(0) * j) - 1];
      }
    }
  } else {
    ptCLoudValid.Color.set_size(0, 0);
  }
  if ((normal.size(0) != 0) && (normal.size(1) != 0)) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r4.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r4[vstride] = j + 1;
        vstride++;
      }
    }
    vstride = normal.size(1);
    ptCLoudValid.Normal.set_size(r4.size(0), normal.size(1));
    xoffset = r4.size(0);
    for (int j{0}; j < vstride; j++) {
      for (int k{0}; k < xoffset; k++) {
        ptCLoudValid.Normal[k + ptCLoudValid.Normal.size(0) * j] =
            normal[(r4[k] + normal.size(0) * j) - 1];
      }
    }
  } else {
    ptCLoudValid.Normal.set_size(0, 0);
  }
  if ((intensity.size(0) != 0) && (intensity.size(1) != 0)) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r5.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r5[vstride] = j + 1;
        vstride++;
      }
    }
    vstride = intensity.size(1);
    ptCLoudValid.Intensity.set_size(r5.size(0), intensity.size(1));
    xoffset = r5.size(0);
    for (int j{0}; j < vstride; j++) {
      for (int k{0}; k < xoffset; k++) {
        ptCLoudValid.Intensity[k + ptCLoudValid.Intensity.size(0) * j] =
            intensity[(r5[k] + intensity.size(0) * j) - 1];
      }
    }
  } else {
    ptCLoudValid.Intensity.set_size(0, 0);
  }
  if ((rangeData.size(0) != 0) && (rangeData.size(1) != 0)) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r6.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r6[vstride] = j + 1;
        vstride++;
      }
    }
    vstride = rangeData.size(1);
    ptCLoudValid.RangeData.set_size(r6.size(0), rangeData.size(1));
    xoffset = r6.size(0);
    for (int j{0}; j < vstride; j++) {
      for (int k{0}; k < xoffset; k++) {
        ptCLoudValid.RangeData[k + ptCLoudValid.RangeData.size(0) * j] =
            rangeData[(r6[k] + rangeData.size(0) * j) - 1];
      }
    }
  } else {
    ptCLoudValid.RangeData.set_size(0, 0);
  }
  ptCLoudValid.Location.set_size(loc.size(0), 3);
  vstride = loc.size(0) * 3;
  for (int j{0}; j < vstride; j++) {
    ptCLoudValid.Location[j] = loc[j];
  }
  ptCLoudValid.PointCloudArrayData.set_size(1, 1);
  ptCLoudValid.PointCloudArrayData[0] = r;
  ptCLoudValid.Kdtree = b_iobj_0;
  ptCLoudValid.matlabCodegenIsDeleted = false;
  if (loc.size(0) == 0) {
    std::memset(&boundingBox[0], 0, 9U * sizeof(double));
  } else {
    double maxHeight;
    double minHeight;
    vstride = loc.size(0);
    b_loc.set_size(loc.size(0));
    for (int j{0}; j < vstride; j++) {
      b_loc[j] = loc[j + loc.size(0) * 2];
    }
    minHeight = internal::minimum(b_loc);
    vstride = loc.size(0);
    b_loc.set_size(loc.size(0));
    for (int j{0}; j < vstride; j++) {
      b_loc[j] = loc[j + loc.size(0) * 2];
    }
    maxHeight = internal::maximum(b_loc);
    vstride = loc.size(0);
    c_loc.set_size(loc.size(0), 2);
    for (int j{0}; j < 2; j++) {
      for (int k{0}; k < vstride; k++) {
        c_loc[k + c_loc.size(0) * j] = loc[k + loc.size(0) * j];
      }
    }
    double boundingBox2d[5];
    findBoundingBox(c_loc, boundingBox2d, &a__1, &a__2);
    boundingBox[0] = boundingBox2d[0];
    boundingBox[1] = boundingBox2d[1];
    boundingBox[2] = (maxHeight + minHeight) / 2.0;
    boundingBox[3] = boundingBox2d[2];
    boundingBox[4] = boundingBox2d[3];
    boundingBox[5] = maxHeight - minHeight;
    boundingBox[6] = 0.0;
    boundingBox[7] = 0.0;
    boundingBox[8] = boundingBox2d[4];
  }
  model = iobj_0;
  for (int j{0}; j < 9; j++) {
    iobj_0->Parameters[j] = boundingBox[j];
  }
  ptCLoudValid.matlabCodegenDestructor();
  lobj_2[0].matlabCodegenDestructor();
  lobj_2[1].matlabCodegenDestructor();
  return model;
}

} // namespace coder

//
// File trailer for pcfitcuboid.cpp
//
// [EOF]
//
