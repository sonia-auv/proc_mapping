//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Tables.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "Tables.h"
#include "Kdtree.h"
#include "combineVectorElements.h"
#include "cuboidModel.h"
#include "find.h"
#include "pcfitcuboid.h"
#include "pcsegdist.h"
#include "pdist.h"
#include "pointCloud.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sonia_common_ObstacleInfoStruct.h"
#include "subspace.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : sonia_common_ObstacleInfoStruct_T *feature
// Return Type  : void
//
void Tables::SegementByAtribute(
    sonia_common_ObstacleInfoStruct_T *feature) const
{
  static const char b_cv[6]{'T', 'a', 'b', 'l', 'e', 's'};
  coder::cuboidModel model;
  coder::pointCloud clusterPT;
  coder::pointCloud *ptCloudOut;
  coder::vision::internal::codegen::Kdtree lobj_2;
  coder::array<double, 2U> b_r;
  coder::array<double, 2U> goodClusters;
  coder::array<double, 2U> loc;
  coder::array<double, 2U> nv;
  coder::array<double, 2U> poses;
  coder::array<double, 1U> intensity;
  coder::array<double, 1U> r1;
  coder::array<int, 2U> r2;
  coder::array<unsigned int, 1U> labels;
  coder::array<int, 1U> r;
  coder::array<unsigned char, 2U> c;
  coder::array<bool, 2U> b_goodClusters;
  coder::array<bool, 1U> b_labels;
  double centerPose[7];
  double absxk;
  double confidence;
  double t;
  double topArea;
  int firstBlockLength;
  int lastBlockLength;
  int nblocks;
  int xblockoffset;
  int xpageoffset;
  clusterPT.matlabCodegenIsDeleted = true;
  //  Separate the pointcloud in different clusters.
  coder::pcsegdist(filteredPT, param.clusterDist, labels, &topArea);
  //  Analyze the clusters.
  firstBlockLength = static_cast<int>(topArea);
  goodClusters.set_size(1, firstBlockLength);
  for (xpageoffset = 0; xpageoffset < firstBlockLength; xpageoffset++) {
    goodClusters[xpageoffset] = 0.0;
  }
  for (xblockoffset = 0; xblockoffset < firstBlockLength; xblockoffset++) {
    nblocks = labels.size(0);
    b_labels.set_size(labels.size(0));
    for (xpageoffset = 0; xpageoffset < nblocks; xpageoffset++) {
      b_labels[xpageoffset] = (labels[xpageoffset] == xblockoffset + 1U);
    }
    coder::b_eml_find(b_labels, r);
    r1.set_size(r.size(0));
    nblocks = r.size(0);
    for (xpageoffset = 0; xpageoffset < nblocks; xpageoffset++) {
      r1[xpageoffset] = r[xpageoffset];
    }
    coder::pcfitcuboid(filteredPT, r1, &model);
    //  Calculate top area.
    topArea = model.Parameters[3];
    absxk = model.Parameters[4];
    topArea *= absxk;
    if ((topArea >= param.topAreaMin) && (topArea <= param.topAreaMax)) {
      topArea = model.Parameters[3];
      absxk = model.Parameters[4];
      if ((topArea > absxk) || (std::isnan(topArea) && (!std::isnan(absxk)))) {
        t = absxk;
      } else {
        t = topArea;
      }
      topArea = model.Parameters[3];
      absxk = model.Parameters[4];
      if ((topArea < absxk) || (std::isnan(topArea) && (!std::isnan(absxk)))) {
        topArea = absxk;
      }
      if (t / topArea > param.squarenessRatio) {
        goodClusters[xblockoffset] = 1.0;
      }
    }
  }
  //  Extract the pose of each good clusters.
  absxk = coder::combineVectorElements(goodClusters);
  xpageoffset = static_cast<int>(absxk);
  poses.set_size(xpageoffset, 7);
  nblocks = static_cast<int>(absxk) * 7;
  for (lastBlockLength = 0; lastBlockLength < nblocks; lastBlockLength++) {
    poses[lastBlockLength] = 0.0;
  }
  for (xblockoffset = 0; xblockoffset < xpageoffset; xblockoffset++) {
    b_goodClusters.set_size(1, goodClusters.size(1));
    nblocks = goodClusters.size(1);
    for (lastBlockLength = 0; lastBlockLength < nblocks; lastBlockLength++) {
      b_goodClusters[lastBlockLength] = (goodClusters[lastBlockLength] == 1.0);
    }
    coder::eml_find(b_goodClusters, r2);
    firstBlockLength = r2[xblockoffset];
    nblocks = labels.size(0);
    b_labels.set_size(labels.size(0));
    for (lastBlockLength = 0; lastBlockLength < nblocks; lastBlockLength++) {
      b_labels[lastBlockLength] =
          (static_cast<double>(labels[lastBlockLength]) == firstBlockLength);
    }
    coder::b_eml_find(b_labels, r);
    r1.set_size(r.size(0));
    nblocks = r.size(0);
    for (lastBlockLength = 0; lastBlockLength < nblocks; lastBlockLength++) {
      r1[lastBlockLength] = r[lastBlockLength];
    }
    filteredPT->subsetImpl(r1, loc, c, nv, intensity, b_r);
    ptCloudOut = clusterPT.init(loc, c, nv, intensity, &lobj_2);
    ptCloudOut->RangeData.set_size(b_r.size(0), b_r.size(1));
    nblocks = b_r.size(0) * b_r.size(1);
    for (lastBlockLength = 0; lastBlockLength < nblocks; lastBlockLength++) {
      ptCloudOut->RangeData[lastBlockLength] = b_r[lastBlockLength];
    }
    clusterPT.get_XLimits(nv);
    clusterPT.get_XLimits(b_r);
    topArea = (nv[0] + b_r[1]) / 2.0;
    clusterPT.get_YLimits(nv);
    clusterPT.get_YLimits(b_r);
    poses[xblockoffset] = topArea;
    poses[xblockoffset + poses.size(0)] = (nv[0] + b_r[1]) / 2.0;
    poses[xblockoffset + poses.size(0) * 2] = param.poseDepth;
    poses[xblockoffset + poses.size(0) * 3] = 1.0;
    poses[xblockoffset + poses.size(0) * 4] = 0.0;
    poses[xblockoffset + poses.size(0) * 5] = 0.0;
    poses[xblockoffset + poses.size(0) * 6] = 0.0;
    clusterPT.matlabCodegenDestructor();
  }
  //  Get the center pose.
  if (poses.size(0) == 0) {
    for (xpageoffset = 0; xpageoffset < 7; xpageoffset++) {
      centerPose[xpageoffset] = 0.0;
    }
  } else {
    if (poses.size(0) <= 1024) {
      firstBlockLength = poses.size(0);
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = poses.size(0) >> 10;
      lastBlockLength = poses.size(0) - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    for (int xi{0}; xi < 7; xi++) {
      xpageoffset = xi * poses.size(0);
      centerPose[xi] = poses[xpageoffset];
      for (int k{2}; k <= firstBlockLength; k++) {
        centerPose[xi] += poses[(xpageoffset + k) - 1];
      }
      for (int ib{2}; ib <= nblocks; ib++) {
        int hi;
        xblockoffset = xpageoffset + ((ib - 1) << 10);
        topArea = poses[xblockoffset];
        if (ib == nblocks) {
          hi = lastBlockLength;
        } else {
          hi = 1024;
        }
        for (int k{2}; k <= hi; k++) {
          topArea += poses[(xblockoffset + k) - 1];
        }
        centerPose[xi] += topArea;
      }
    }
  }
  for (xpageoffset = 0; xpageoffset < 7; xpageoffset++) {
    centerPose[xpageoffset] /= static_cast<double>(poses.size(0));
  }
  //  Verify confidence
  confidence = 100.0;
  //  Reduce de confidence if we see more than 3 tables or only one
  //  table.
  if ((absxk > 3.0) || (absxk == 1.0)) {
    confidence = 25.0;
  }
  //  Calculate the distance between two cluster if wee saw 2
  //  potential clusters only.
  if (absxk == 25.0) {
    double b_poses[6];
    b_poses[0] = poses[0];
    b_poses[1] = poses[1];
    b_poses[2] = poses[poses.size(0)];
    b_poses[3] = poses[poses.size(0) + 1];
    b_poses[4] = poses[poses.size(0) * 2];
    b_poses[5] = poses[poses.size(0) * 2 + 1];
    if (coder::pdist(b_poses) > param.maxBetweenDist) {
      confidence *= 0.25;
    }
  }
  //  Check if the 3 tables are aligned if we exactly 3 tables.
  if (absxk == 3.0) {
    double UB[2];
    double varargin_1[2];
    double w[2];
    double nrmUB;
    //  Vector 1
    w[0] = poses[1] - poses[0];
    w[1] = poses[poses.size(0) + 1] - poses[poses.size(0)];
    coder::modified_orth(w, varargin_1, &firstBlockLength);
    w[0] = poses[2] - poses[0];
    w[1] = poses[poses.size(0) + 2] - poses[poses.size(0)];
    coder::modified_orth(w, UB, &nblocks);
    for (int k{0}; k < firstBlockLength; k++) {
      for (lastBlockLength = 0; lastBlockLength < nblocks; lastBlockLength++) {
        w[lastBlockLength] = varargin_1[0] * UB[0] + varargin_1[1] * UB[1];
      }
      for (lastBlockLength = 0; lastBlockLength < nblocks; lastBlockLength++) {
        topArea = w[lastBlockLength];
        UB[0] -= varargin_1[0] * topArea;
        UB[1] -= varargin_1[1] * topArea;
      }
    }
    if (nblocks + 1 <= 1) {
      UB[0] = 0.0;
      UB[1] = 0.0;
    }
    topArea = 3.3121686421112381E-170;
    absxk = std::abs(UB[0]);
    if (absxk > 3.3121686421112381E-170) {
      nrmUB = 1.0;
      topArea = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      nrmUB = t * t;
    }
    absxk = std::abs(UB[1]);
    if (absxk > topArea) {
      t = topArea / absxk;
      nrmUB = nrmUB * t * t + 1.0;
      topArea = absxk;
    } else {
      t = absxk / topArea;
      nrmUB += t * t;
    }
    nrmUB = topArea * std::sqrt(nrmUB);
    if (nrmUB > 1.0) {
      nrmUB = 1.0;
    }
    if (std::asin(nrmUB) > 0.4) {
      confidence *= 0.5;
    }
  }
  //  Return the obstacle info.
  sonia_common_ObstacleInfoStruct(feature);
  feature->IsValid = true;
  feature->Name.set_size(1, 6);
  for (xpageoffset = 0; xpageoffset < 6; xpageoffset++) {
    feature->Name[xpageoffset] = b_cv[xpageoffset];
  }
  feature->Confidence = static_cast<float>(confidence);
  feature->Pose.Position.X = centerPose[0];
  feature->Pose.Position.Y = centerPose[1];
  feature->Pose.Position.Z = centerPose[2];
  feature->Pose.Orientation.W = 1.0;
  feature->Pose.Orientation.X = 0.0;
  feature->Pose.Orientation.Y = 0.0;
  feature->Pose.Orientation.Z = 0.0;
}

//
// File trailer for Tables.cpp
//
// [EOF]
//
