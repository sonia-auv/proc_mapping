//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcfitplane.cpp
//
// Code generation for function 'pcfitplane'
//

// Include files
#include "pcfitplane.h"
#include "Kdtree.h"
#include "blockedSummation.h"
#include "find.h"
#include "mod.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "proc_mapping_rtwutil.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Declarations
namespace coder {
static void evalPlane(const ::coder::array<double, 2U> &model,
                      const ::coder::array<double, 2U> &points,
                      ::coder::array<double, 1U> &dis);

}

// Function Definitions
namespace coder {
static void evalPlane(const ::coder::array<double, 2U> &model,
                      const ::coder::array<double, 2U> &points,
                      ::coder::array<double, 1U> &dis)
{
  array<double, 1U> y;
  int nx;
  nx = points.size(0);
  y.set_size(points.size(0));
  for (int i{0}; i < nx; i++) {
    y[i] = (points[i] * model[0] + points[points.size(0) + i] * model[1]) +
           points[(points.size(0) << 1) + i] * model[2];
  }
  nx = y.size(0);
  for (int i{0}; i < nx; i++) {
    y[i] = y[i] + model[3];
  }
  nx = y.size(0);
  dis.set_size(y.size(0));
  for (int i{0}; i < nx; i++) {
    dis[i] = std::abs(y[i]);
  }
}

void pcfitplane(const pointCloud *varargin_1, double varargin_2,
                planeModel *iobj_0, planeModel **model,
                ::coder::array<double, 1U> &inlierIndices,
                ::coder::array<double, 1U> &outlierIndices,
                ::coder::array<double, 2U> &meanError)
{
  pointCloud pc;
  pointCloud *b_pc;
  vision::internal::codegen::Kdtree lobj_1;
  array<double, 2U> b_modelParams;
  array<double, 2U> indices;
  array<double, 2U> location;
  array<double, 2U> modelParams;
  array<double, 2U> samplePoints;
  array<double, 1U> hashTbl;
  array<double, 1U> link;
  array<double, 1U> loc;
  array<int, 1U> r2;
  array<int, 1U> val;
  array<int, 1U> validPtCloudIndices;
  array<unsigned char, 2U> color;
  array<bool, 2U> r;
  array<bool, 1U> bestInliers;
  array<bool, 1U> r1;
  array<bool, 1U> x;
  int status;
  int vlen;
  int y;
  pc.matlabCodegenIsDeleted = true;
  varargin_1->extractValidPoints(location, color, modelParams, hashTbl,
                                 b_modelParams, bestInliers);
  b_pc = pc.init(location, color, modelParams, hashTbl, &lobj_1);
  b_pc->RangeData.set_size(b_modelParams.size(0), b_modelParams.size(1));
  vlen = b_modelParams.size(0) * b_modelParams.size(1);
  for (y = 0; y < vlen; y++) {
    b_pc->RangeData[y] = b_modelParams[y];
  }
  b_eml_find(bestInliers, validPtCloudIndices);
  vlen = b_pc->Location.size(0);
  status = (vlen < 3);
  modelParams.set_size(1, 4);
  modelParams[0] = 0.0;
  modelParams[1] = 0.0;
  modelParams[2] = 0.0;
  modelParams[3] = 0.0;
  if (status == 0) {
    double bestDis;
    int idxTrial;
    int numPts;
    int numTrials;
    int skipTrials;
    bool b_y;
    bool exitg1;
    location.set_size(pc.Location.size(0), 3);
    vlen = pc.Location.size(0) * 3;
    for (y = 0; y < vlen; y++) {
      location[y] = pc.Location[y];
    }
    numPts = location.size(0);
    idxTrial = 1;
    numTrials = 1000;
    bestDis = varargin_2 * static_cast<double>(location.size(0));
    modelParams.set_size(0, 0);
    skipTrials = 0;
    bestInliers.set_size(location.size(0));
    vlen = location.size(0);
    for (y = 0; y < vlen; y++) {
      bestInliers[y] = false;
    }
    while ((idxTrial <= numTrials) && (skipTrials < 10000)) {
      double a[3];
      double b[3];
      double i;
      double j;
      double normal_idx_1;
      double normal_idx_2;
      double selectedLoc;
      double t;
      indices.set_size(1, 3);
      indices[1] = 0.0;
      indices[2] = 0.0;
      if (numPts <= 3) {
        indices[0] = 1.0;
        for (int k{0}; k <= numPts - 2; k++) {
          j = b_rand() * ((static_cast<double>(k) + 1.0) + 1.0);
          j = std::floor(j);
          indices[k + 1] = indices[static_cast<int>(j + 1.0) - 1];
          indices[static_cast<int>(j + 1.0) - 1] =
              (static_cast<double>(k) + 1.0) + 1.0;
        }
      } else if (static_cast<double>(numPts) / 4.0 <= 3.0) {
        t = 0.0;
        j = numPts;
        selectedLoc = 3.0 / static_cast<double>(numPts);
        i = b_rand();
        while (i > selectedLoc) {
          t++;
          j--;
          selectedLoc += (1.0 - selectedLoc) * (3.0 / j);
        }
        t++;
        j = b_rand();
        j = std::floor(j);
        indices[0] = 0.0;
        indices[static_cast<int>(j + 1.0) - 1] = t;
        j = static_cast<double>(numPts) - t;
        selectedLoc = 2.0 / j;
        i = b_rand();
        while (i > selectedLoc) {
          t++;
          j--;
          selectedLoc += (1.0 - selectedLoc) * (2.0 / j);
        }
        t++;
        j = b_rand() * 2.0;
        j = std::floor(j);
        indices[1] = indices[static_cast<int>(j + 1.0) - 1];
        indices[static_cast<int>(j + 1.0) - 1] = t;
        j = static_cast<double>(numPts) - t;
        selectedLoc = 1.0 / j;
        i = b_rand();
        while (i > selectedLoc) {
          t++;
          j--;
          selectedLoc += (1.0 - selectedLoc) * (1.0 / j);
        }
        t++;
        j = b_rand() * 3.0;
        j = std::floor(j);
        indices[2] = indices[static_cast<int>(j + 1.0) - 1];
        indices[static_cast<int>(j + 1.0) - 1] = t;
      } else {
        hashTbl.set_size(3);
        link.set_size(3);
        val.set_size(3);
        loc.set_size(3);
        hashTbl[0] = 0.0;
        hashTbl[1] = 0.0;
        link[1] = 0.0;
        val[1] = 0;
        loc[1] = 0.0;
        hashTbl[2] = 0.0;
        link[2] = 0.0;
        val[2] = 0;
        loc[2] = 0.0;
        selectedLoc = b_rand() * ((static_cast<double>(numPts) - 1.0) + 1.0);
        selectedLoc = std::floor(selectedLoc);
        indices[0] = selectedLoc + 1.0;
        loc[0] = selectedLoc;
        link[0] = 0.0;
        hashTbl[static_cast<int>(b_mod(selectedLoc) + 1.0) - 1] = 1.0;
        selectedLoc =
            hashTbl[static_cast<int>(b_mod(static_cast<double>(numPts) - 1.0) +
                                     1.0) -
                    1];
        while ((selectedLoc > 0.0) &&
               (loc[0] != static_cast<double>(numPts) - 1.0)) {
          selectedLoc = 0.0;
        }
        if (selectedLoc > 0.0) {
          val[0] = 0;
        } else {
          val[0] = numPts - 1;
        }
        selectedLoc = b_rand() * ((static_cast<double>(numPts) - 2.0) + 1.0);
        selectedLoc = std::floor(selectedLoc);
        i = b_mod(selectedLoc) + 1.0;
        j = hashTbl[static_cast<int>(i) - 1];
        while ((j > 0.0) && (loc[0] != selectedLoc)) {
          j = 0.0;
        }
        if (j > 0.0) {
          indices[1] = static_cast<double>(val[0]) + 1.0;
        } else {
          indices[1] = selectedLoc + 1.0;
          j = 2.0;
          loc[1] = selectedLoc;
          link[1] = hashTbl[static_cast<int>(i) - 1];
          hashTbl[static_cast<int>(i) - 1] = 2.0;
        }
        selectedLoc =
            hashTbl[static_cast<int>(b_mod(static_cast<double>(numPts) - 2.0) +
                                     1.0) -
                    1];
        while ((selectedLoc > 0.0) && (loc[static_cast<int>(selectedLoc) - 1] !=
                                       static_cast<double>(numPts) - 2.0)) {
          selectedLoc = link[static_cast<int>(selectedLoc) - 1];
        }
        if (selectedLoc > 0.0) {
          val[static_cast<int>(j) - 1] = val[static_cast<int>(selectedLoc) - 1];
        } else {
          val[static_cast<int>(j) - 1] = numPts - 2;
        }
        selectedLoc = b_rand() * ((static_cast<double>(numPts) - 3.0) + 1.0);
        selectedLoc = std::floor(selectedLoc);
        j = hashTbl[static_cast<int>(b_mod(selectedLoc) + 1.0) - 1];
        while ((j > 0.0) && (loc[static_cast<int>(j) - 1] != selectedLoc)) {
          j = link[static_cast<int>(j) - 1];
        }
        if (j > 0.0) {
          indices[2] = static_cast<double>(val[static_cast<int>(j) - 1]) + 1.0;
        } else {
          indices[2] = selectedLoc + 1.0;
        }
      }
      samplePoints.set_size(3, 3);
      for (y = 0; y < 3; y++) {
        for (vlen = 0; vlen < 3; vlen++) {
          samplePoints[vlen + samplePoints.size(0) * y] = location
              [(static_cast<int>(indices[vlen]) + location.size(0) * y) - 1];
        }
        selectedLoc = samplePoints[samplePoints.size(0) * y];
        a[y] = samplePoints[samplePoints.size(0) * y + 1] - selectedLoc;
        b[y] = samplePoints[samplePoints.size(0) * y + 2] - selectedLoc;
      }
      t = a[1] * b[2] - b[1] * a[2];
      normal_idx_1 = b[0] * a[2] - a[0] * b[2];
      normal_idx_2 = a[0] * b[1] - b[0] * a[1];
      j = (t * t + normal_idx_1 * normal_idx_1) + normal_idx_2 * normal_idx_2;
      if (j < 2.2204460492503131E-16) {
        b_modelParams.set_size(0, 0);
      } else {
        selectedLoc = std::sqrt(j);
        i = t / selectedLoc;
        j = -samplePoints[0] * i;
        t = i;
        i = normal_idx_1 / selectedLoc;
        j += -samplePoints[samplePoints.size(0)] * i;
        normal_idx_1 = i;
        i = normal_idx_2 / selectedLoc;
        j += -samplePoints[samplePoints.size(0) * 2] * i;
        b_modelParams.set_size(1, 4);
        b_modelParams[0] = t;
        b_modelParams[1] = normal_idx_1;
        b_modelParams[2] = i;
        b_modelParams[3] = j;
      }
      x.set_size(b_modelParams.size(0) * b_modelParams.size(1));
      vlen = b_modelParams.size(0) * b_modelParams.size(1);
      for (y = 0; y < vlen; y++) {
        x[y] = std::isinf(b_modelParams[y]);
      }
      r1.set_size(b_modelParams.size(0) * b_modelParams.size(1));
      vlen = b_modelParams.size(0) * b_modelParams.size(1);
      for (y = 0; y < vlen; y++) {
        r1[y] = std::isnan(b_modelParams[y]);
      }
      vlen = x.size(0);
      for (y = 0; y < vlen; y++) {
        x[y] = ((!x[y]) && (!r1[y]));
      }
      b_y = true;
      vlen = 1;
      exitg1 = false;
      while ((!exitg1) && (vlen <= x.size(0))) {
        if (!x[vlen - 1]) {
          b_y = false;
          exitg1 = true;
        } else {
          vlen++;
        }
      }
      if ((b_modelParams.size(0) * b_modelParams.size(1) == 4) && b_y) {
        evalPlane(b_modelParams, location, hashTbl);
        y = hashTbl.size(0);
        for (int k{0}; k < y; k++) {
          if (hashTbl[k] > varargin_2) {
            hashTbl[k] = varargin_2;
          }
        }
        selectedLoc = blockedSummation(hashTbl, hashTbl.size(0));
        if (selectedLoc < bestDis) {
          bestDis = selectedLoc;
          bestInliers.set_size(hashTbl.size(0));
          vlen = hashTbl.size(0);
          for (y = 0; y < vlen; y++) {
            bestInliers[y] = (hashTbl[y] < varargin_2);
          }
          modelParams.set_size(b_modelParams.size(0), b_modelParams.size(1));
          vlen = b_modelParams.size(0) * b_modelParams.size(1);
          for (y = 0; y < vlen; y++) {
            modelParams[y] = b_modelParams[y];
          }
          x.set_size(hashTbl.size(0));
          vlen = hashTbl.size(0);
          for (y = 0; y < vlen; y++) {
            x[y] = (hashTbl[y] < varargin_2);
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
          selectedLoc = rt_powd_snf(
              static_cast<double>(y) / static_cast<double>(numPts), 3.0);
          if (selectedLoc < 2.2204460492503131E-16) {
            vlen = MAX_int32_T;
          } else {
            selectedLoc =
                std::ceil(-1.9999999999999996 / std::log10(1.0 - selectedLoc));
            if (selectedLoc < 2.147483648E+9) {
              vlen = static_cast<int>(selectedLoc);
            } else if (selectedLoc >= 2.147483648E+9) {
              vlen = MAX_int32_T;
            } else {
              vlen = 0;
            }
          }
          if (numTrials > vlen) {
            numTrials = vlen;
          }
        }
        idxTrial++;
      } else {
        skipTrials++;
      }
    }
    x.set_size(modelParams.size(0) * modelParams.size(1));
    vlen = modelParams.size(0) * modelParams.size(1);
    for (y = 0; y < vlen; y++) {
      x[y] = std::isinf(modelParams[y]);
    }
    r1.set_size(modelParams.size(0) * modelParams.size(1));
    vlen = modelParams.size(0) * modelParams.size(1);
    for (y = 0; y < vlen; y++) {
      r1[y] = std::isnan(modelParams[y]);
    }
    vlen = x.size(0);
    for (y = 0; y < vlen; y++) {
      x[y] = ((!x[y]) && (!r1[y]));
    }
    b_y = true;
    vlen = 1;
    exitg1 = false;
    while ((!exitg1) && (vlen <= x.size(0))) {
      if (!x[vlen - 1]) {
        b_y = false;
        exitg1 = true;
      } else {
        vlen++;
      }
    }
    if ((modelParams.size(0) * modelParams.size(1) == 4) && b_y &&
        (bestInliers.size(0) != 0)) {
      vlen = bestInliers.size(0);
      y = bestInliers[0];
      for (int k{2}; k <= vlen; k++) {
        y += bestInliers[k - 1];
      }
      if (y < 3) {
        status = 2;
      }
    } else {
      status = 2;
    }
  }
  if ((modelParams.size(0) == 0) || (modelParams.size(1) == 0)) {
    modelParams.set_size(1, 4);
    modelParams[0] = 0.0;
    modelParams[1] = 0.0;
    modelParams[2] = 0.0;
    modelParams[3] = 0.0;
  }
  *model = iobj_0;
  iobj_0->Parameters.set_size(1, modelParams.size(1));
  vlen = modelParams.size(1);
  for (y = 0; y < vlen; y++) {
    iobj_0->Parameters[y] = modelParams[y];
  }
  if (status == 0) {
    evalPlane(modelParams, pc.Location, hashTbl);
    r.set_size(hashTbl.size(0), 1);
    vlen = hashTbl.size(0);
    for (y = 0; y < vlen; y++) {
      r[y] = (hashTbl[y] < varargin_2);
    }
    y = r.size(0) - 1;
    vlen = 0;
    for (int k{0}; k <= y; k++) {
      if (r[k]) {
        vlen++;
      }
    }
    inlierIndices.set_size(vlen);
    vlen = 0;
    for (int k{0}; k <= y; k++) {
      if (r[k]) {
        inlierIndices[vlen] = validPtCloudIndices[k];
        vlen++;
      }
    }
    vlen = varargin_1->Location.size(0);
    bestInliers.set_size(vlen);
    for (y = 0; y < vlen; y++) {
      bestInliers[y] = true;
    }
    validPtCloudIndices.set_size(inlierIndices.size(0));
    vlen = inlierIndices.size(0);
    for (y = 0; y < vlen; y++) {
      validPtCloudIndices[y] = static_cast<int>(inlierIndices[y]);
    }
    vlen = validPtCloudIndices.size(0);
    for (y = 0; y < vlen; y++) {
      bestInliers[validPtCloudIndices[y] - 1] = false;
    }
    b_eml_find(bestInliers, validPtCloudIndices);
    outlierIndices.set_size(validPtCloudIndices.size(0));
    vlen = validPtCloudIndices.size(0);
    for (y = 0; y < vlen; y++) {
      outlierIndices[y] = validPtCloudIndices[y];
    }
    r.set_size(hashTbl.size(0), 1);
    vlen = hashTbl.size(0);
    for (y = 0; y < vlen; y++) {
      r[y] = (hashTbl[y] < varargin_2);
    }
    y = r.size(0) - 1;
    vlen = 0;
    for (int k{0}; k <= y; k++) {
      if (r[k]) {
        vlen++;
      }
    }
    r2.set_size(vlen);
    vlen = 0;
    for (int k{0}; k <= y; k++) {
      if (r[k]) {
        r2[vlen] = k + 1;
        vlen++;
      }
    }
    link.set_size(r2.size(0));
    vlen = r2.size(0);
    for (y = 0; y < vlen; y++) {
      link[y] = hashTbl[r2[y] - 1];
    }
    meanError.set_size(1, 1);
    meanError[0] =
        blockedSummation(link, r2.size(0)) / static_cast<double>(r2.size(0));
  } else {
    inlierIndices.set_size(0);
    outlierIndices.set_size(0);
    meanError.set_size(0, 0);
  }
}

} // namespace coder

// End of code generation (pcfitplane.cpp)
