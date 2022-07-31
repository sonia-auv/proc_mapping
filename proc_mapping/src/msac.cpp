//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: msac.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "msac.h"
#include "blockedSummation.h"
#include "mod.h"
#include "pcfitplane.h"
#include "proc_mapping_rtwutil.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &allPoints
//                double params_maxDistance
//                const ::coder::array<double, 2U> &varargin_1
//                bool *isFound
//                ::coder::array<double, 2U> &bestModelParams
// Return Type  : void
//
namespace coder {
namespace vision {
namespace internal {
namespace ransac {
void msac(const ::coder::array<double, 2U> &allPoints,
          double params_maxDistance,
          const ::coder::array<double, 2U> &varargin_1, bool *isFound,
          ::coder::array<double, 2U> &bestModelParams)
{
  array<double, 2U> indices;
  array<double, 2U> modelParams;
  array<double, 2U> samplePoints;
  array<double, 1U> hashTbl;
  array<double, 1U> link;
  array<double, 1U> loc;
  array<int, 1U> val;
  array<bool, 1U> bestInliers;
  array<bool, 1U> r;
  array<bool, 1U> x;
  double angle;
  double bestDis;
  int idxTrial;
  int numPts;
  int numTrials;
  int skipTrials;
  int vlen;
  int y;
  bool exitg1;
  bool isValidModel;
  numPts = allPoints.size(0);
  idxTrial = 1;
  numTrials = 1000;
  bestDis = params_maxDistance * static_cast<double>(allPoints.size(0));
  bestModelParams.set_size(0, 0);
  skipTrials = 0;
  bestInliers.set_size(allPoints.size(0));
  vlen = allPoints.size(0);
  for (y = 0; y < vlen; y++) {
    bestInliers[y] = false;
  }
  while ((idxTrial <= numTrials) && (skipTrials < 10000)) {
    double a[3];
    double b[3];
    double i;
    double j;
    double newEntry;
    double normal_idx_0;
    double normal_idx_1;
    double normal_idx_2;
    indices.set_size(1, 3);
    for (y = 0; y < 3; y++) {
      indices[y] = 0.0;
    }
    if (numPts <= 3) {
      indices[0] = 1.0;
      for (y = 0; y <= numPts - 2; y++) {
        j = b_rand() * ((static_cast<double>(y) + 1.0) + 1.0);
        j = std::floor(j);
        indices[y + 1] = indices[static_cast<int>(j + 1.0) - 1];
        indices[static_cast<int>(j + 1.0) - 1] =
            (static_cast<double>(y) + 1.0) + 1.0;
      }
    } else if (static_cast<double>(numPts) / 4.0 <= 3.0) {
      newEntry = 0.0;
      for (y = 0; y < 3; y++) {
        j = static_cast<double>(numPts) - newEntry;
        angle = static_cast<double>(3 - y) / j;
        i = b_rand();
        while (i > angle) {
          newEntry++;
          j--;
          angle += (1.0 - angle) * (static_cast<double>(3 - y) / j);
        }
        newEntry++;
        j = b_rand() * (static_cast<double>(y) + 1.0);
        j = std::floor(j);
        indices[y] = indices[static_cast<int>(j + 1.0) - 1];
        indices[static_cast<int>(j + 1.0) - 1] = newEntry;
      }
    } else {
      hashTbl.set_size(3);
      link.set_size(3);
      val.set_size(3);
      loc.set_size(3);
      for (y = 0; y < 3; y++) {
        hashTbl[y] = 0.0;
        link[y] = 0.0;
        val[y] = 0;
        loc[y] = 0.0;
      }
      newEntry = 1.0;
      for (y = 0; y < 3; y++) {
        vlen = (numPts - y) - 1;
        angle = b_rand() * (static_cast<double>(vlen) + 1.0);
        angle = std::floor(angle);
        i = b_mod(angle) + 1.0;
        j = hashTbl[static_cast<int>(i) - 1];
        while ((j > 0.0) && (loc[static_cast<int>(j) - 1] != angle)) {
          j = link[static_cast<int>(j) - 1];
        }
        if (j > 0.0) {
          indices[y] = static_cast<double>(val[static_cast<int>(j) - 1]) + 1.0;
        } else {
          indices[y] = angle + 1.0;
          j = newEntry;
          newEntry++;
          loc[static_cast<int>(j) - 1] = angle;
          link[static_cast<int>(j) - 1] = hashTbl[static_cast<int>(i) - 1];
          hashTbl[static_cast<int>(i) - 1] = j;
        }
        if (y + 1 < 3) {
          angle =
              hashTbl[static_cast<int>(b_mod(static_cast<double>(vlen)) + 1.0) -
                      1];
          while ((angle > 0.0) && (loc[static_cast<int>(angle) - 1] != vlen)) {
            angle = link[static_cast<int>(angle) - 1];
          }
          if (angle > 0.0) {
            val[static_cast<int>(j) - 1] = val[static_cast<int>(angle) - 1];
          } else {
            val[static_cast<int>(j) - 1] = vlen;
          }
        }
      }
    }
    samplePoints.set_size(indices.size(1), 3);
    vlen = indices.size(1);
    for (y = 0; y < 3; y++) {
      for (int k{0}; k < vlen; k++) {
        samplePoints[k + samplePoints.size(0) * y] =
            allPoints[(static_cast<int>(indices[k]) + allPoints.size(0) * y) -
                      1];
      }
      angle = samplePoints[samplePoints.size(0) * y];
      a[y] = samplePoints[samplePoints.size(0) * y + 1] - angle;
      b[y] = samplePoints[samplePoints.size(0) * y + 2] - angle;
    }
    normal_idx_0 = a[1] * b[2] - b[1] * a[2];
    normal_idx_1 = b[0] * a[2] - a[0] * b[2];
    normal_idx_2 = a[0] * b[1] - b[0] * a[1];
    j = (normal_idx_0 * normal_idx_0 + normal_idx_1 * normal_idx_1) +
        normal_idx_2 * normal_idx_2;
    if (j < 2.2204460492503131E-16) {
      modelParams.set_size(0, 0);
    } else {
      angle = std::sqrt(j);
      i = normal_idx_0 / angle;
      newEntry = -samplePoints[0] * i;
      normal_idx_0 = i;
      i = normal_idx_1 / angle;
      newEntry += -samplePoints[samplePoints.size(0)] * i;
      normal_idx_1 = i;
      i = normal_idx_2 / angle;
      newEntry += -samplePoints[samplePoints.size(0) * 2] * i;
      modelParams.set_size(1, 4);
      modelParams[0] = normal_idx_0;
      modelParams[1] = normal_idx_1;
      modelParams[2] = i;
      modelParams[3] = newEntry;
    }
    x.set_size(modelParams.size(0) * modelParams.size(1));
    vlen = modelParams.size(0) * modelParams.size(1);
    for (y = 0; y < vlen; y++) {
      x[y] = std::isinf(modelParams[y]);
    }
    r.set_size(modelParams.size(0) * modelParams.size(1));
    vlen = modelParams.size(0) * modelParams.size(1);
    for (y = 0; y < vlen; y++) {
      r[y] = std::isnan(modelParams[y]);
    }
    vlen = x.size(0);
    for (y = 0; y < vlen; y++) {
      x[y] = ((!x[y]) && (!r[y]));
    }
    isValidModel = true;
    vlen = 1;
    exitg1 = false;
    while ((!exitg1) && (vlen <= x.size(0))) {
      if (!x[vlen - 1]) {
        isValidModel = false;
        exitg1 = true;
      } else {
        vlen++;
      }
    }
    isValidModel =
        ((modelParams.size(0) * modelParams.size(1) == 4) && isValidModel);
    if (isValidModel) {
      angle = std::abs(std::acos(
          std::fmin(1.0, std::fmax(-1.0, (varargin_1[0] * modelParams[0] +
                                          varargin_1[1] * modelParams[1]) +
                                             varargin_1[2] * modelParams[2]))));
      angle = std::fmin(angle, 3.1415926535897931 - angle);
      isValidModel = (angle < 0.78539816339744828);
    }
    if (isValidModel) {
      evalPlane(modelParams, allPoints, hashTbl);
      vlen = hashTbl.size(0);
      for (y = 0; y < vlen; y++) {
        if (hashTbl[y] > params_maxDistance) {
          hashTbl[y] = params_maxDistance;
        }
      }
      angle = blockedSummation(hashTbl, hashTbl.size(0));
      if (angle < bestDis) {
        bestDis = angle;
        bestInliers.set_size(hashTbl.size(0));
        vlen = hashTbl.size(0);
        for (y = 0; y < vlen; y++) {
          bestInliers[y] = (hashTbl[y] < params_maxDistance);
        }
        bestModelParams.set_size(modelParams.size(0), modelParams.size(1));
        vlen = modelParams.size(0) * modelParams.size(1);
        for (y = 0; y < vlen; y++) {
          bestModelParams[y] = modelParams[y];
        }
        x.set_size(hashTbl.size(0));
        vlen = hashTbl.size(0);
        for (y = 0; y < vlen; y++) {
          x[y] = (hashTbl[y] < params_maxDistance);
        }
        vlen = x.size(0);
        if (x.size(0) == 0) {
          y = 0;
        } else {
          y = x[0];
          for (int k{2}; k <= vlen; k++) {
            y += x[k - 1];
          }
        }
        angle = rt_powd_snf(
            static_cast<double>(y) / static_cast<double>(numPts), 3.0);
        if (angle < 2.2204460492503131E-16) {
          vlen = MAX_int32_T;
        } else {
          angle = std::ceil(-1.9999999999999996 / std::log10(1.0 - angle));
          if (angle < 2.147483648E+9) {
            vlen = static_cast<int>(angle);
          } else if (angle >= 2.147483648E+9) {
            vlen = MAX_int32_T;
          } else {
            vlen = 0;
          }
        }
        if (numTrials > vlen) {
          numTrials = vlen;
        }
      }
      idxTrial = static_cast<int>(idxTrial + 1U);
    } else {
      skipTrials++;
    }
  }
  x.set_size(bestModelParams.size(0) * bestModelParams.size(1));
  vlen = bestModelParams.size(0) * bestModelParams.size(1);
  for (y = 0; y < vlen; y++) {
    x[y] = std::isinf(bestModelParams[y]);
  }
  r.set_size(bestModelParams.size(0) * bestModelParams.size(1));
  vlen = bestModelParams.size(0) * bestModelParams.size(1);
  for (y = 0; y < vlen; y++) {
    r[y] = std::isnan(bestModelParams[y]);
  }
  vlen = x.size(0);
  for (y = 0; y < vlen; y++) {
    x[y] = ((!x[y]) && (!r[y]));
  }
  isValidModel = true;
  vlen = 1;
  exitg1 = false;
  while ((!exitg1) && (vlen <= x.size(0))) {
    if (!x[vlen - 1]) {
      isValidModel = false;
      exitg1 = true;
    } else {
      vlen++;
    }
  }
  isValidModel = ((bestModelParams.size(0) * bestModelParams.size(1) == 4) &&
                  isValidModel);
  if (isValidModel) {
    angle = std::abs(std::acos(std::fmin(
        1.0, std::fmax(-1.0, (varargin_1[0] * bestModelParams[0] +
                              varargin_1[1] * bestModelParams[1]) +
                                 varargin_1[2] * bestModelParams[2]))));
    angle = std::fmin(angle, 3.1415926535897931 - angle);
    isValidModel = (angle < 0.78539816339744828);
  }
  if (isValidModel && (bestInliers.size(0) != 0)) {
    vlen = bestInliers.size(0);
    y = bestInliers[0];
    for (int k{2}; k <= vlen; k++) {
      y += bestInliers[k - 1];
    }
    if (y >= 3) {
      *isFound = true;
    } else {
      *isFound = false;
    }
  } else {
    *isFound = false;
  }
}

} // namespace ransac
} // namespace internal
} // namespace vision
} // namespace coder

//
// File trailer for msac.cpp
//
// [EOF]
//
