//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Buoys.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "Buoys.h"
#include "Kdtree.h"
#include "PointCloudSegmentation.h"
#include "affine3d.h"
#include "combineVectorElements.h"
#include "find.h"
#include "isRigidTransform.h"
#include "mtimes.h"
#include "pcfitplane.h"
#include "pcregistericp.h"
#include "pcsegdist.h"
#include "pdist.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "proc_mapping_types.h"
#include "quat2rotm.h"
#include "quatUtilities.h"
#include "rigid3d.h"
#include "rotm2quat.h"
#include "rt_nonfinite.h"
#include "sonia_common_ObstacleInfoStruct.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Declarations
static coder::vision::internal::codegen::Kdtree
b_binary_expand_op(const Buoys *in1, const coder::array<unsigned int, 1U> &in2,
                   const coder::array<int, 2U> &in3,
                   coder::vision::internal::codegen::Kdtree in4,
                   coder::pointCloud *in5);

// Function Definitions
//
// Apply Ransac
//
// Arguments    : coder::pointCloud *subPT
//                const double auvQuat[4]
//                double p[3]
//                double q[4]
//                double *confidence
// Return Type  : void
//
void Buoys::getBuoyPose(coder::pointCloud *subPT, const double auvQuat[4],
                        double p[3], double q[4], double *confidence) const
{
  coder::b_pointCloud buoyTformed;
  coder::i_pointCloud lobj_0;
  coder::planeModel model;
  coder::planeModel *b_model;
  coder::pointCloud plane;
  coder::pointCloud *ptCloudOut;
  coder::rigid3d tformICP;
  coder::vision::internal::codegen::Kdtree lobj_3;
  coder::vision::internal::codegen::Kdtree lobj_4;
  coder::array<double, 2U> loc;
  coder::array<double, 2U> meanError;
  coder::array<double, 2U> r;
  coder::array<double, 1U> a__3;
  coder::array<double, 1U> indexOnPlane;
  coder::array<float, 2U> b_loc;
  coder::array<float, 2U> b_nv;
  coder::array<float, 2U> nv;
  coder::array<float, 1U> c_loc;
  coder::array<unsigned char, 2U> c;
  double T[16];
  double b_varargin_1[16];
  double varargin_1[9];
  double qApprox[4];
  double box[3];
  double absxk;
  double qnrm;
  double t;
  double y;
  float b_T[16];
  int boffset;
  int coffset;
  int loop_ub;
  bool b;
  bool b1;
  plane.matlabCodegenIsDeleted = true;
  buoyTformed.matlabCodegenIsDeleted = true;
  coder::pcfitplane(subPT, param.planeTol, &model, &b_model, indexOnPlane, a__3,
                    meanError);
  subPT->subsetImpl(indexOnPlane, loc, c, meanError, a__3, r);
  ptCloudOut = plane.init(loc, c, meanError, a__3, &lobj_4);
  ptCloudOut->RangeData.set_size(r.size(0), r.size(1));
  loop_ub = r.size(0) * r.size(1);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    ptCloudOut->RangeData[boffset] = r[boffset];
  }
  //  Get ransac plane pose approximation
  quatUtilities::getOrientedPointOnPlanarFace(&model, subPT, box, qApprox);
  //  Transform the buoy on the plane.
  qnrm = ((qApprox[0] * qApprox[0] + qApprox[1] * qApprox[1]) +
          qApprox[2] * qApprox[2]) +
         qApprox[3] * qApprox[3];
  qApprox[0] /= qnrm;
  qApprox[1] = -qApprox[1] / qnrm;
  qApprox[2] = -qApprox[2] / qnrm;
  qApprox[3] = -qApprox[3] / qnrm;
  coder::quat2rotm(qApprox, varargin_1);
  for (boffset = 0; boffset < 3; boffset++) {
    loop_ub = boffset << 2;
    T[loop_ub] = varargin_1[3 * boffset];
    T[loop_ub + 1] = varargin_1[3 * boffset + 1];
    T[loop_ub + 2] = varargin_1[3 * boffset + 2];
    T[boffset + 12] = 0.0;
    T[loop_ub + 3] = box[boffset];
  }
  T[15] = 1.0;
  coder::rigid3d::isTransformationMatrixRigid(T);
  for (boffset = 0; boffset < 3; boffset++) {
    loop_ub = boffset << 2;
    b_varargin_1[loop_ub] = varargin_1[3 * boffset];
    b_varargin_1[loop_ub + 1] = varargin_1[3 * boffset + 1];
    b_varargin_1[loop_ub + 2] = varargin_1[3 * boffset + 2];
    b_varargin_1[boffset + 12] = 0.0;
    b_varargin_1[loop_ub + 3] = box[boffset];
  }
  b_varargin_1[15] = 1.0;
  coder::rigid3d::isTransformationMatrixRigid(b_varargin_1);
  for (boffset = 0; boffset < 16; boffset++) {
    b_T[boffset] = static_cast<float>(T[boffset]);
  }
  lobj_0.matlabCodegenIsDeleted = true;
  nv.set_size(buoyPT->Location.size(0), 3);
  loop_ub = buoyPT->Location.size(0) * 3;
  for (boffset = 0; boffset < loop_ub; boffset++) {
    nv[boffset] = buoyPT->Location[boffset];
  }
  loop_ub = nv.size(0);
  b_loc.set_size(nv.size(0), 3);
  for (int j{0}; j < 3; j++) {
    coffset = j * loop_ub;
    boffset = j * 3;
    for (int i{0}; i < loop_ub; i++) {
      b_loc[coffset + i] =
          (nv[i] * b_T[boffset % 3 + (div_nzp_s32_floor(boffset, 3) << 2)] +
           nv[nv.size(0) + i] * b_T[(boffset + 1) % 3 +
                                    (div_nzp_s32_floor(boffset + 1, 3) << 2)]) +
          nv[(nv.size(0) << 1) + i] *
              b_T[(boffset + 2) % 3 + (div_nzp_s32_floor(boffset + 2, 3) << 2)];
    }
  }
  loop_ub = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (boffset = 0; boffset <= loop_ub; boffset++) {
    c_loc[boffset] = b_loc[boffset] + b_T[3];
  }
  loop_ub = c_loc.size(0);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    b_loc[boffset] = c_loc[boffset];
  }
  loop_ub = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (boffset = 0; boffset <= loop_ub; boffset++) {
    c_loc[boffset] = b_loc[boffset + b_loc.size(0)] + b_T[7];
  }
  loop_ub = c_loc.size(0);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    b_loc[boffset + b_loc.size(0)] = c_loc[boffset];
  }
  loop_ub = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (boffset = 0; boffset <= loop_ub; boffset++) {
    c_loc[boffset] = b_loc[boffset + b_loc.size(0) * 2] + b_T[11];
  }
  loop_ub = c_loc.size(0);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    b_loc[boffset + b_loc.size(0) * 2] = c_loc[boffset];
  }
  nv.set_size(0, 0);
  b = (buoyPT->Normal.size(0) == 0);
  b1 = (buoyPT->Normal.size(1) == 0);
  if ((!b) && (!b1)) {
    if (coder::vision::internal::isRigidTransform(b_T)) {
      float c_T[9];
      nv.set_size(buoyPT->Normal.size(0), buoyPT->Normal.size(1));
      loop_ub = buoyPT->Normal.size(0) * buoyPT->Normal.size(1);
      for (boffset = 0; boffset < loop_ub; boffset++) {
        nv[boffset] = buoyPT->Normal[boffset];
      }
      for (boffset = 0; boffset < 3; boffset++) {
        loop_ub = boffset << 2;
        c_T[3 * boffset] = b_T[loop_ub];
        c_T[3 * boffset + 1] = b_T[loop_ub + 1];
        c_T[3 * boffset + 2] = b_T[loop_ub + 2];
      }
      b_nv.set_size(nv.size(0), nv.size(1));
      loop_ub = nv.size(0) * nv.size(1) - 1;
      for (boffset = 0; boffset <= loop_ub; boffset++) {
        b_nv[boffset] = nv[boffset];
      }
      coder::internal::blas::mtimes(b_nv, c_T, nv);
    } else {
      lobj_0.Location.set_size(b_loc.size(0), 3);
      loop_ub = b_loc.size(0) * 3;
      for (boffset = 0; boffset < loop_ub; boffset++) {
        lobj_0.Location[boffset] = b_loc[boffset];
      }
      lobj_0.Color.set_size(0, 0);
      lobj_0.Normal.set_size(0, 0);
      lobj_0.Intensity.set_size(0, 0);
      lobj_0.b_Kdtree = lobj_0._pobj0.init();
      lobj_0.matlabCodegenIsDeleted = false;
      lobj_0.surfaceNormalImpl(nv);
    }
  }
  buoyTformed.init(b_loc, buoyPT->Color, nv, buoyPT->Intensity, &lobj_3);
  lobj_0.matlabCodegenDestructor();
  //  Apply icp.
  coder::pcregistericp(&buoyTformed, &plane, param.icpInlierRatio, &tformICP);
  //  Get buoys transformation.
  for (boffset = 0; boffset < 4; boffset++) {
    qnrm = T[boffset];
    absxk = T[boffset + 4];
    t = T[boffset + 8];
    y = T[boffset + 12];
    for (coffset = 0; coffset < 4; coffset++) {
      loop_ub = coffset << 2;
      b_varargin_1[boffset + loop_ub] =
          ((qnrm * tformICP.AffineTform.T[loop_ub] +
            absxk * tformICP.AffineTform.T[loop_ub + 1]) +
           t * tformICP.AffineTform.T[loop_ub + 2]) +
          y * tformICP.AffineTform.T[loop_ub + 3];
    }
  }
  coder::rigid3d::isTransformationMatrixRigid(b_varargin_1);
  coder::rigid3d::isTransformationMatrixRigid(b_varargin_1);
  //  Verify plane confidence
  //  Get Z normal
  box[2] = model.Parameters[2];
  //  Ratio in plane
  loop_ub = subPT->Location.size(0);
  if (indexOnPlane.size(0) < 1) {
    boffset = 1;
  } else {
    boffset = indexOnPlane.size(0);
  }
  *confidence = 100.0 * (1.0 - std::abs(box[2])) *
                (static_cast<double>(boffset) / static_cast<double>(loop_ub));
  //  Teturn transformation and confidence
  for (boffset = 0; boffset < 3; boffset++) {
    p[boffset] = b_varargin_1[(boffset << 2) + 3];
    varargin_1[3 * boffset] = b_varargin_1[boffset];
    varargin_1[3 * boffset + 1] = b_varargin_1[boffset + 4];
    varargin_1[3 * boffset + 2] = b_varargin_1[boffset + 8];
  }
  coder::rotm2quat(varargin_1, qApprox);
  // =================================================================
  //  Fonction qui calcule l'angle entre 2 quaternion
  q[0] = qApprox[0];
  q[1] = qApprox[1];
  q[2] = qApprox[2];
  q[3] = qApprox[3];
  qnrm = 3.3121686421112381E-170;
  absxk = std::abs((qApprox[0] * auvQuat[1] + auvQuat[0] * -qApprox[1]) +
                   (-qApprox[2] * auvQuat[3] - auvQuat[2] * -qApprox[3]));
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    qnrm = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = std::abs((qApprox[0] * auvQuat[2] + auvQuat[0] * -qApprox[2]) +
                   (auvQuat[1] * -qApprox[3] - -qApprox[1] * auvQuat[3]));
  if (absxk > qnrm) {
    t = qnrm / absxk;
    y = y * t * t + 1.0;
    qnrm = absxk;
  } else {
    t = absxk / qnrm;
    y += t * t;
  }
  absxk = std::abs((qApprox[0] * auvQuat[3] + auvQuat[0] * -qApprox[3]) +
                   (-qApprox[1] * auvQuat[2] - auvQuat[1] * -qApprox[2]));
  if (absxk > qnrm) {
    t = qnrm / absxk;
    y = y * t * t + 1.0;
    qnrm = absxk;
  } else {
    t = absxk / qnrm;
    y += t * t;
  }
  y = qnrm * std::sqrt(y);
  qnrm = std::abs(2.0 * rt_atan2d_snf(y, ((qApprox[0] * auvQuat[0] -
                                           -qApprox[1] * auvQuat[1]) -
                                          -qApprox[2] * auvQuat[2]) -
                                             -qApprox[3] * auvQuat[3]));
  if (std::fmin(6.2831853071795862 - qnrm, qnrm) > 1.5707963267948966) {
    q[0] = ((qApprox[0] * 6.123233995736766E-17 - qApprox[1] * 0.0) -
            qApprox[2] * 0.0) -
           qApprox[3];
    q[1] = (qApprox[0] * 0.0 + 6.123233995736766E-17 * qApprox[1]) +
           (qApprox[2] - 0.0 * qApprox[3]);
    q[2] = (qApprox[0] * 0.0 + 6.123233995736766E-17 * qApprox[2]) +
           (0.0 * qApprox[3] - qApprox[1]);
    q[3] = (qApprox[0] + 6.123233995736766E-17 * qApprox[3]) +
           (qApprox[1] * 0.0 - 0.0 * qApprox[2]);
  }
  //  Extract bounding box
  PointCloudSegmentation::objectAllignBoudingBox(q, &plane, box);
  //  Find area
  qnrm = box[1] * box[2];
  if ((qnrm < param.minArea) || (qnrm > param.maxArea)) {
    *confidence /= 2.0;
  }
  // pcshow(buoyTformed)
}

//
// Arguments    : const Buoys *in1
//                const coder::array<unsigned int, 1U> &in2
//                const coder::array<int, 2U> &in3
//                coder::vision::internal::codegen::Kdtree in4
//                coder::pointCloud *in5
// Return Type  : coder::vision::internal::codegen::Kdtree
//
static coder::vision::internal::codegen::Kdtree
b_binary_expand_op(const Buoys *in1, const coder::array<unsigned int, 1U> &in2,
                   const coder::array<int, 2U> &in3,
                   coder::vision::internal::codegen::Kdtree in4,
                   coder::pointCloud *in5)
{
  coder::array<bool, 2U> b_in2;
  int aux_1_1;
  int i;
  int in2_idx_0;
  int loop_ub;
  int stride_1_1;
  in2_idx_0 = in2.size(0);
  if (in3.size(1) == 1) {
    i = 1;
  } else {
    i = in3.size(1);
  }
  b_in2.set_size(in2_idx_0, i);
  stride_1_1 = (in3.size(1) != 1);
  aux_1_1 = 0;
  if (in3.size(1) == 1) {
    loop_ub = 1;
  } else {
    loop_ub = in3.size(1);
  }
  for (i = 0; i < loop_ub; i++) {
    for (int i1{0}; i1 < in2_idx_0; i1++) {
      b_in2[i1 + b_in2.size(0) * i] =
          (static_cast<double>(in2[i1]) == in3[aux_1_1]);
    }
    aux_1_1 += stride_1_1;
  }
  in1->filteredPT->b_select(b_in2, &in4, in5);
  return in4;
}

//
// Get clusters
//
// Arguments    : const double auvPose[7]
//                sonia_common_ObstacleInfoStruct_T feature[2]
// Return Type  : void
//
void Buoys::SegementByAtribute(const double auvPose[7],
                               sonia_common_ObstacleInfoStruct_T feature[2])
{
  static const char b_cv[5]{'B', 'u', 'o', 'y', 's'};
  Buoys b_this;
  coder::planeModel model;
  coder::planeModel *b_model;
  coder::pointCloud b_clusterPT;
  coder::pointCloud c_clusterPT;
  coder::pointCloud clusterPT;
  coder::pointCloud plane;
  coder::pointCloud *ptCloudOut;
  coder::vision::internal::codegen::Kdtree lobj_3[2];
  coder::vision::internal::codegen::Kdtree b_lobj_3;
  coder::vision::internal::codegen::Kdtree lobj_2;
  coder::vision::internal::codegen::Kdtree *iobj_0;
  coder::array<double, 2U> b_r;
  coder::array<double, 2U> distances;
  coder::array<double, 2U> goodCluster;
  coder::array<double, 2U> loc;
  coder::array<double, 2U> meanError;
  coder::array<double, 1U> a__2;
  coder::array<double, 1U> indexOnPlane;
  coder::array<int, 2U> b_index;
  coder::array<unsigned int, 1U> r;
  coder::array<unsigned char, 2U> c;
  coder::array<bool, 2U> b_goodCluster;
  coder::array<bool, 2U> r1;
  double p[6];
  double q[4];
  double b_p[3];
  double box[3];
  double area;
  double offset_idx_2;
  int b_loop_ub;
  int c_loop_ub;
  int d_loop_ub;
  int i;
  int last;
  int loop_ub;
  int loop_ub_tmp;
  int unnamed_idx_0;
  clusterPT.matlabCodegenIsDeleted = true;
  b_clusterPT.matlabCodegenIsDeleted = true;
  coder::pcsegdist(filteredPT, param.clusterDist, r, &area);
  PTlabels.set_size(r.size(0), 1);
  loop_ub = r.size(0);
  for (i = 0; i < loop_ub; i++) {
    PTlabels[i] = r[i];
  }
  loop_ub_tmp = static_cast<int>(area);
  goodCluster.set_size(1, loop_ub_tmp);
  sonia_common_ObstacleInfoStruct(&feature[1]);
  //  Get the good clusters.
  if (static_cast<int>(area) - 1 >= 0) {
    unnamed_idx_0 = r.size(0);
    b_loop_ub = r.size(0);
  }
  for (int b_i{0}; b_i < loop_ub_tmp; b_i++) {
    bool b;
    //  Extract pointCloud
    plane.matlabCodegenIsDeleted = true;
    c_clusterPT.matlabCodegenIsDeleted = true;
    r1.set_size(unnamed_idx_0, 1);
    for (i = 0; i < b_loop_ub; i++) {
      r1[i] = (r[i] == b_i + 1U);
    }
    filteredPT->b_select(r1, &lobj_3[0], &c_clusterPT);
    //  Fit plane on cluster
    coder::pcfitplane(&c_clusterPT, param.planeTol, &model, &b_model,
                      indexOnPlane, a__2, meanError);
    iobj_0 = &lobj_3[1];
    c_clusterPT.subsetImpl(indexOnPlane, loc, c, meanError, a__2, b_r);
    ptCloudOut = plane.init(loc, c, meanError, a__2, iobj_0);
    ptCloudOut->RangeData.set_size(b_r.size(0), b_r.size(1));
    loop_ub = b_r.size(0) * b_r.size(1);
    for (i = 0; i < loop_ub; i++) {
      ptCloudOut->RangeData[i] = b_r[i];
    }
    b = (plane.Location.size(0) == 0);
    if (b) {
      goodCluster[b_i] = 0.0;
    } else {
      //  Get Z normal
      offset_idx_2 = model.Parameters[2];
      //  Ratio in plane
      last = c_clusterPT.Location.size(0);
      //  Extract pose of the plane.
      quatUtilities::getOrientedPointOnPlanarFace(&model, &plane, b_p, q);
      //  Extract bounding box
      PointCloudSegmentation::objectAllignBoudingBox(q, &plane, box);
      //  Find area
      area = box[1] * box[2];
      //  Check if cluster is a potential buoys
      if (std::abs(offset_idx_2) < param.zNormalThres) {
        if (indexOnPlane.size(0) < 1) {
          i = 1;
        } else {
          i = indexOnPlane.size(0);
        }
        if ((static_cast<double>(i) / static_cast<double>(last) >
             param.inPlaneThres) &&
            (area > param.minArea) && (area < param.maxArea)) {
          goodCluster[b_i] = 1.0;
        } else {
          goodCluster[b_i] = 0.0;
        }
      } else {
        goodCluster[b_i] = 0.0;
      }
    }
    c_clusterPT.matlabCodegenDestructor();
    plane.matlabCodegenDestructor();
  }
  area = coder::combineVectorElements(goodCluster);
  loop_ub = static_cast<int>(coder::combineVectorElements(goodCluster));
  i = static_cast<int>(area);
  distances.set_size(1, i);
  for (i = 0; i < loop_ub; i++) {
    distances[i] = 0.0;
  }
  if (static_cast<int>(area) - 1 >= 0) {
    c_loop_ub = goodCluster.size(1);
    unnamed_idx_0 = r.size(0);
    d_loop_ub = r.size(0);
  }
  if (loop_ub - 1 >= 0) {
    p[1] = auvPose[0];
    p[3] = auvPose[1];
    p[5] = auvPose[2];
  }
  for (int b_i{0}; b_i < loop_ub; b_i++) {
    b_goodCluster.set_size(1, goodCluster.size(1));
    for (i = 0; i < c_loop_ub; i++) {
      b_goodCluster[i] = (goodCluster[i] == 1.0);
    }
    coder::eml_find(b_goodCluster, b_index);
    last = b_index[b_i];
    r1.set_size(unnamed_idx_0, 1);
    for (i = 0; i < d_loop_ub; i++) {
      r1[i] = (static_cast<double>(r[i]) == last);
    }
    filteredPT->b_select(r1, &b_lobj_3, &b_clusterPT);
    b_this = *this;
    b_this.getBuoyPose(&b_clusterPT, *(double(*)[4]) & auvPose[3], b_p, q,
                       &area);
    p[0] = b_p[0];
    p[2] = b_p[1];
    p[4] = b_p[2];
    distances[b_i] = coder::pdist(p);
    b_clusterPT.matlabCodegenDestructor();
  }
  last = distances.size(1);
  if (distances.size(1) <= 2) {
    if (distances.size(1) == 1) {
      loop_ub = 1;
    } else if ((distances[0] > distances[distances.size(1) - 1]) ||
               (std::isnan(distances[0]) &&
                (!std::isnan(distances[distances.size(1) - 1])))) {
      loop_ub = distances.size(1);
    } else {
      loop_ub = 1;
    }
  } else {
    if (!std::isnan(distances[0])) {
      loop_ub = 1;
    } else {
      bool exitg1;
      loop_ub = 0;
      b_loop_ub = 2;
      exitg1 = false;
      while ((!exitg1) && (b_loop_ub <= last)) {
        if (!std::isnan(distances[b_loop_ub - 1])) {
          loop_ub = b_loop_ub;
          exitg1 = true;
        } else {
          b_loop_ub++;
        }
      }
    }
    if (loop_ub == 0) {
      loop_ub = 1;
    } else {
      area = distances[loop_ub - 1];
      i = loop_ub + 1;
      for (b_loop_ub = i; b_loop_ub <= last; b_loop_ub++) {
        offset_idx_2 = distances[b_loop_ub - 1];
        if (area > offset_idx_2) {
          area = offset_idx_2;
          loop_ub = b_loop_ub;
        }
      }
    }
  }
  goodCluster.set_size(1, loop_ub_tmp);
  for (i = 0; i < loop_ub_tmp; i++) {
    goodCluster[i] = 0.0;
  }
  goodCluster[loop_ub - 1] = 1.0;
  //  Keep the closest cluster.
  b_goodCluster.set_size(1, goodCluster.size(1));
  loop_ub = goodCluster.size(1);
  for (i = 0; i < loop_ub; i++) {
    b_goodCluster[i] = (goodCluster[i] == 1.0);
  }
  coder::eml_find(b_goodCluster, b_index);
  if (b_index.size(1) == 1) {
    unnamed_idx_0 = r.size(0);
    r1.set_size(r.size(0), 1);
    for (i = 0; i < unnamed_idx_0; i++) {
      r1[i] = (static_cast<double>(r[i]) == b_index[0]);
    }
    filteredPT->b_select(r1, &lobj_2, &clusterPT);
  } else {
    lobj_2 = b_binary_expand_op(this, r, b_index, lobj_2, &clusterPT);
  }
  b_this = *this;
  b_this.getBuoyPose(&clusterPT, *(double(*)[4]) & auvPose[3], b_p, q, &area);
  feature[1].IsValid = true;
  feature[1].Name.set_size(1, 5);
  for (i = 0; i < 5; i++) {
    feature[1].Name[i] = b_cv[i];
  }
  double a;
  double b_a;
  double offset_idx_0;
  double offset_idx_1;
  double q_idx_1;
  double q_idx_2;
  double q_idx_3;
  feature[1].Confidence = static_cast<float>(area);
  box[1] = param.gap / 2.0;
  area = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
  offset_idx_2 = q[0] / area;
  q_idx_1 = -q[1] / area;
  q_idx_2 = -q[2] / area;
  q_idx_3 = -q[3] / area;
  // =================================================================
  //  Fonction qui tourne un vecteur selon un quaternion.
  //  quaternion partie scalaire
  //  quaternion partie vectoriel
  //  QuatRotate n'est pas compilable
  a = 2.0 * ((q_idx_1 * 0.0 + box[1] * q_idx_2) + q_idx_3 * 0.0);
  b_a = offset_idx_2 * offset_idx_2 -
        ((q_idx_1 * q_idx_1 + q_idx_2 * q_idx_2) + q_idx_3 * q_idx_3);
  area = 2.0 * offset_idx_2;
  offset_idx_0 =
      (a * q_idx_1 + b_a * 0.0) + area * (q_idx_2 * 0.0 - box[1] * q_idx_3);
  offset_idx_1 =
      (a * q_idx_2 + b_a * box[1]) + area * (q_idx_3 * 0.0 - q_idx_1 * 0.0);
  offset_idx_2 =
      (a * q_idx_3 + b_a * 0.0) + area * (q_idx_1 * box[1] - q_idx_2 * 0.0);
  feature[1].Pose.Position.X = b_p[0] + offset_idx_0;
  feature[1].Pose.Position.Y = b_p[1] + offset_idx_1;
  feature[1].Pose.Position.Z = b_p[2] + offset_idx_2;
  feature[1].Pose.Orientation.W = q[0];
  feature[1].Pose.Orientation.X = q[1];
  feature[1].Pose.Orientation.Y = q[2];
  feature[1].Pose.Orientation.Z = q[3];
  feature[0] = feature[1];
  feature[1].Pose.Position.X = b_p[0] - offset_idx_0;
  feature[1].Pose.Position.Y = b_p[1] - offset_idx_1;
  feature[1].Pose.Position.Z = b_p[2] - offset_idx_2;
}

//
// File trailer for Buoys.cpp
//
// [EOF]
//
