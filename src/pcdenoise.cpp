//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcdenoise.cpp
//
// Code generation for function 'pcdenoise'
//

// Include files
#include "pcdenoise.h"
#include "Kdtree.h"
#include "blockedSummation.h"
#include "combineVectorElements.h"
#include "find.h"
#include "pointCloud.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
pointCloud *pcdenoise(c_pointCloud *ptCloudIn,
                      vision::internal::codegen::Kdtree *iobj_0,
                      pointCloud *iobj_1)
{
  pointCloud *ptCloudOut;
  vision::internal::codegen::b_Kdtree *b_this;
  array<double, 2U> dists;
  array<double, 2U> finiteMeanDist;
  array<double, 2U> inputData;
  array<double, 2U> meanDist;
  array<double, 2U> nv;
  array<double, 2U> points;
  array<double, 2U> rangeData;
  array<double, 1U> absdiff;
  array<double, 1U> b_finiteMeanDist;
  array<double, 1U> intensity;
  array<unsigned int, 2U> indices;
  array<unsigned int, 2U> inlierIndices;
  array<int, 2U> r2;
  array<unsigned int, 1U> valid;
  array<int, 1U> y;
  array<unsigned char, 2U> c;
  array<unsigned char, 2U> color;
  array<bool, 2U> b_meanDist;
  array<bool, 2U> r;
  array<bool, 2U> r1;
  array<bool, 2U> x;
  array<bool, 1U> isValidPoints;
  double bsum;
  int firstBlockLength;
  int i;
  int i1;
  unsigned int maxval;
  int nblocks;
  int npages;
  unsigned int qY;
  points.set_size(ptCloudIn->Location.size(0), 3);
  firstBlockLength = ptCloudIn->Location.size(0) * 3;
  for (i = 0; i < firstBlockLength; i++) {
    points[i] = ptCloudIn->Location[i];
  }
  npages = ptCloudIn->Location.size(0);
  if (!ptCloudIn->b_Kdtree->IsIndexed) {
    b_this = ptCloudIn->b_Kdtree;
    inputData.set_size(ptCloudIn->Location.size(0), 3);
    firstBlockLength = ptCloudIn->Location.size(0) * 3;
    for (i = 0; i < firstBlockLength; i++) {
      inputData[i] = ptCloudIn->Location[i];
    }
    b_this->buildIndex(inputData);
    b_this->IsIndexed = true;
  }
  ptCloudIn->b_Kdtree->knnSearch(points,
                                 std::fmin(5.0, static_cast<double>(npages)),
                                 indices, dists, valid);
  npages = valid.size(0);
  maxval = valid[0];
  for (int k{2}; k <= npages; k++) {
    qY = valid[k - 1];
    if (maxval < qY) {
      maxval = qY;
    }
  }
  qY = maxval - 1U;
  if (maxval - 1U > maxval) {
    qY = 0U;
  }
  if (qY + 1U < 2U) {
    i = -1;
    i1 = -1;
  } else {
    i = 0;
    i1 = static_cast<int>(qY);
  }
  i1 -= i;
  if ((i1 == 0) || (dists.size(1) == 0) || (i1 == 0)) {
    meanDist.set_size(1, dists.size(1));
    firstBlockLength = dists.size(1);
    for (i = 0; i < firstBlockLength; i++) {
      meanDist[i] = 0.0;
    }
  } else {
    int lastBlockLength;
    npages = dists.size(1) - 1;
    meanDist.set_size(1, dists.size(1));
    if (i1 <= 1024) {
      firstBlockLength = i1;
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = i1 >> 10;
      lastBlockLength = i1 - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    for (int xi{0}; xi <= npages; xi++) {
      int i2;
      int xpageoffset;
      xpageoffset = xi * i1;
      meanDist[xi] =
          dists[((i + xpageoffset % i1) +
                 dists.size(0) * div_nzp_s32_floor(xpageoffset, i1)) +
                1];
      for (int k{2}; k <= firstBlockLength; k++) {
        i2 = (xpageoffset + k) - 1;
        meanDist[xi] =
            meanDist[xi] +
            dists[((i + i2 % i1) + dists.size(0) * div_nzp_s32_floor(i2, i1)) +
                  1];
      }
      for (int ib{2}; ib <= nblocks; ib++) {
        int hi;
        int xblockoffset;
        xblockoffset = xpageoffset + ((ib - 1) << 10);
        bsum = dists[((i + xblockoffset % i1) +
                      dists.size(0) * div_nzp_s32_floor(xblockoffset, i1)) +
                     1];
        if (ib == nblocks) {
          hi = lastBlockLength;
        } else {
          hi = 1024;
        }
        for (int k{2}; k <= hi; k++) {
          i2 = (xblockoffset + k) - 1;
          bsum += dists[((i + i2 % i1) +
                         dists.size(0) * div_nzp_s32_floor(i2, i1)) +
                        1];
        }
        meanDist[xi] = meanDist[xi] + bsum;
      }
    }
  }
  meanDist.set_size(1, meanDist.size(1));
  firstBlockLength = meanDist.size(1) - 1;
  for (i = 0; i <= firstBlockLength; i++) {
    meanDist[i] = meanDist[i] / static_cast<double>(qY);
  }
  x.set_size(points.size(0), 3);
  firstBlockLength = points.size(0) * 3;
  for (i = 0; i < firstBlockLength; i++) {
    x[i] = std::isinf(points[i]);
  }
  r.set_size(points.size(0), 3);
  firstBlockLength = points.size(0) * 3;
  for (i = 0; i < firstBlockLength; i++) {
    r[i] = std::isnan(points[i]);
  }
  firstBlockLength = x.size(0) * 3;
  x.set_size(x.size(0), 3);
  for (i = 0; i < firstBlockLength; i++) {
    x[i] = ((!x[i]) && (!r[i]));
  }
  if (x.size(0) == 0) {
    y.set_size(0);
  } else {
    npages = x.size(0);
    y.set_size(x.size(0));
    for (nblocks = 0; nblocks < npages; nblocks++) {
      y[nblocks] = x[nblocks];
    }
    for (int k{0}; k < 2; k++) {
      firstBlockLength = (k + 1) * npages;
      for (nblocks = 0; nblocks < npages; nblocks++) {
        y[nblocks] = y[nblocks] + x[firstBlockLength + nblocks];
      }
    }
  }
  isValidPoints.set_size(y.size(0));
  firstBlockLength = y.size(0);
  for (i = 0; i < firstBlockLength; i++) {
    isValidPoints[i] = (y[i] == 3);
  }
  firstBlockLength = isValidPoints.size(0);
  for (nblocks = 0; nblocks < firstBlockLength; nblocks++) {
    if (!isValidPoints[nblocks]) {
      meanDist[nblocks] = rtNaN;
    }
  }
  b_meanDist.set_size(1, meanDist.size(1));
  firstBlockLength = meanDist.size(1);
  for (i = 0; i < firstBlockLength; i++) {
    b_meanDist[i] = std::isinf(meanDist[i]);
  }
  r1.set_size(1, meanDist.size(1));
  firstBlockLength = meanDist.size(1);
  for (i = 0; i < firstBlockLength; i++) {
    r1[i] = std::isnan(meanDist[i]);
  }
  b_meanDist.set_size(1, b_meanDist.size(1));
  firstBlockLength = b_meanDist.size(1) - 1;
  for (i = 0; i <= firstBlockLength; i++) {
    b_meanDist[i] = ((!b_meanDist[i]) && (!r1[i]));
  }
  firstBlockLength = b_meanDist.size(1) - 1;
  npages = 0;
  for (nblocks = 0; nblocks <= firstBlockLength; nblocks++) {
    if (b_meanDist[nblocks]) {
      npages++;
    }
  }
  finiteMeanDist.set_size(1, npages);
  npages = 0;
  for (nblocks = 0; nblocks <= firstBlockLength; nblocks++) {
    if (b_meanDist[nblocks]) {
      finiteMeanDist[npages] = meanDist[nblocks];
      npages++;
    }
  }
  firstBlockLength = finiteMeanDist.size(1);
  if (finiteMeanDist.size(1) == 0) {
    bsum = rtNaN;
  } else if (finiteMeanDist.size(1) == 1) {
    if ((!std::isinf(finiteMeanDist[0])) && (!std::isnan(finiteMeanDist[0]))) {
      bsum = 0.0;
    } else {
      bsum = rtNaN;
    }
  } else {
    double scale;
    npages = finiteMeanDist.size(1);
    b_finiteMeanDist = finiteMeanDist.reshape(npages);
    bsum = blockedSummation(b_finiteMeanDist, finiteMeanDist.size(1)) /
           static_cast<double>(finiteMeanDist.size(1));
    absdiff.set_size(finiteMeanDist.size(1));
    for (int k{0}; k < firstBlockLength; k++) {
      absdiff[k] = std::abs(finiteMeanDist[k] - bsum);
    }
    bsum = 0.0;
    scale = 3.3121686421112381E-170;
    npages = finiteMeanDist.size(1);
    for (int k{0}; k < npages; k++) {
      if (absdiff[k] > scale) {
        double t;
        t = scale / absdiff[k];
        bsum = bsum * t * t + 1.0;
        scale = absdiff[k];
      } else {
        double t;
        t = absdiff[k] / scale;
        bsum += t * t;
      }
    }
    bsum = scale * std::sqrt(bsum);
    bsum /= std::sqrt(static_cast<double>(finiteMeanDist.size(1)) - 1.0);
  }
  bsum += combineVectorElements(finiteMeanDist) /
          static_cast<double>(finiteMeanDist.size(1));
  b_meanDist.set_size(1, meanDist.size(1));
  firstBlockLength = meanDist.size(1);
  for (i = 0; i < firstBlockLength; i++) {
    b_meanDist[i] = (meanDist[i] <= bsum);
  }
  eml_find(b_meanDist, r2);
  inlierIndices.set_size(1, r2.size(1));
  firstBlockLength = r2.size(1);
  for (i = 0; i < firstBlockLength; i++) {
    i1 = r2[i];
    if (i1 < 0) {
      i1 = 0;
    }
    inlierIndices[i] = static_cast<unsigned int>(i1);
  }
  points.set_size(ptCloudIn->Location.size(0), 3);
  firstBlockLength = ptCloudIn->Location.size(0) * 3;
  for (i = 0; i < firstBlockLength; i++) {
    points[i] = ptCloudIn->Location[i];
  }
  color.set_size(ptCloudIn->Color.size(0), ptCloudIn->Color.size(1));
  firstBlockLength = ptCloudIn->Color.size(0) * ptCloudIn->Color.size(1);
  for (i = 0; i < firstBlockLength; i++) {
    color[i] = ptCloudIn->Color[i];
  }
  dists.set_size(ptCloudIn->Normal.size(0), ptCloudIn->Normal.size(1));
  firstBlockLength = ptCloudIn->Normal.size(0) * ptCloudIn->Normal.size(1);
  for (i = 0; i < firstBlockLength; i++) {
    dists[i] = ptCloudIn->Normal[i];
  }
  absdiff.set_size(ptCloudIn->Intensity.size(0));
  firstBlockLength = ptCloudIn->Intensity.size(0);
  for (i = 0; i < firstBlockLength; i++) {
    absdiff[i] = ptCloudIn->Intensity[i];
  }
  rangeData.set_size(ptCloudIn->RangeData.size(0),
                     ptCloudIn->RangeData.size(1));
  firstBlockLength =
      ptCloudIn->RangeData.size(0) * ptCloudIn->RangeData.size(1);
  for (i = 0; i < firstBlockLength; i++) {
    rangeData[i] = ptCloudIn->RangeData[i];
  }
  if (points.size(0) != 0) {
    inputData.set_size(inlierIndices.size(1), 3);
    firstBlockLength = inlierIndices.size(1);
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < firstBlockLength; i1++) {
        inputData[i1 + inputData.size(0) * i] =
            points[(static_cast<int>(inlierIndices[i1]) + points.size(0) * i) -
                   1];
      }
    }
  } else {
    inputData.set_size(0, 3);
  }
  if ((color.size(0) != 0) && (color.size(1) != 0)) {
    firstBlockLength = color.size(1);
    c.set_size(inlierIndices.size(1), color.size(1));
    npages = inlierIndices.size(1);
    for (i = 0; i < firstBlockLength; i++) {
      for (i1 = 0; i1 < npages; i1++) {
        c[i1 + c.size(0) * i] =
            color[(static_cast<int>(inlierIndices[i1]) + color.size(0) * i) -
                  1];
      }
    }
  } else {
    c.set_size(0, 0);
  }
  if ((dists.size(0) != 0) && (dists.size(1) != 0)) {
    firstBlockLength = dists.size(1);
    nv.set_size(inlierIndices.size(1), dists.size(1));
    npages = inlierIndices.size(1);
    for (i = 0; i < firstBlockLength; i++) {
      for (i1 = 0; i1 < npages; i1++) {
        nv[i1 + nv.size(0) * i] =
            dists[(static_cast<int>(inlierIndices[i1]) + dists.size(0) * i) -
                  1];
      }
    }
  } else {
    nv.set_size(0, 0);
  }
  if (absdiff.size(0) != 0) {
    intensity.set_size(inlierIndices.size(1));
    firstBlockLength = inlierIndices.size(1);
    for (i = 0; i < firstBlockLength; i++) {
      intensity[i] = absdiff[static_cast<int>(inlierIndices[i]) - 1];
    }
  } else {
    intensity.set_size(0);
  }
  if ((rangeData.size(0) != 0) && (rangeData.size(1) != 0)) {
    firstBlockLength = rangeData.size(1);
    dists.set_size(inlierIndices.size(1), rangeData.size(1));
    npages = inlierIndices.size(1);
    for (i = 0; i < firstBlockLength; i++) {
      for (i1 = 0; i1 < npages; i1++) {
        dists[i1 + dists.size(0) * i] = rangeData
            [(static_cast<int>(inlierIndices[i1]) + rangeData.size(0) * i) - 1];
      }
    }
  } else {
    dists.set_size(0, 0);
  }
  ptCloudOut = iobj_1->init(inputData, c, nv, intensity, iobj_0);
  ptCloudOut->RangeData.set_size(dists.size(0), dists.size(1));
  firstBlockLength = dists.size(0) * dists.size(1);
  for (i = 0; i < firstBlockLength; i++) {
    ptCloudOut->RangeData[i] = dists[i];
  }
  return ptCloudOut;
}

} // namespace coder

// End of code generation (pcdenoise.cpp)
