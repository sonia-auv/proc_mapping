//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Buoys.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
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
#include "proc_mapping_data.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "proc_mapping_types.h"
#include "quat2rotm.h"
#include "quatUtilities.h"
#include "rigid3d.h"
#include "rt_nonfinite.h"
#include "schur.h"
#include "sonia_common_ObstacleInfoStruct.h"
#include "xzggev.h"
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
  creal_T V[16];
  creal_T alpha1[4];
  double K[16];
  double T[16];
  double varargin_1[9];
  double qApprox[4];
  double box[3];
  double K12;
  double K13;
  double K14;
  double K23;
  double K24;
  float b_T[16];
  int boffset;
  int coffset;
  int i;
  int j;
  int sgn;
  bool b;
  bool b_p;
  bool exitg2;
  plane.matlabCodegenIsDeleted = true;
  buoyTformed.matlabCodegenIsDeleted = true;
  coder::pcfitplane(subPT, param.planeTol, &model, &b_model, indexOnPlane, a__3,
                    meanError);
  subPT->subsetImpl(indexOnPlane, loc, c, meanError, a__3, r);
  ptCloudOut = plane.init(loc, c, meanError, a__3, &lobj_4);
  ptCloudOut->RangeData.set_size(r.size(0), r.size(1));
  sgn = r.size(0) * r.size(1);
  for (i = 0; i < sgn; i++) {
    ptCloudOut->RangeData[i] = r[i];
  }
  //  Get ransac plane pose approximation
  quatUtilities::getOrientedPointOnPlanarFace(&model, subPT, box, qApprox);
  //  Transform the buoy on the plane.
  coder::quat2rotm(qApprox, varargin_1);
  for (i = 0; i < 3; i++) {
    sgn = i << 2;
    T[sgn] = varargin_1[3 * i];
    T[sgn + 1] = varargin_1[3 * i + 1];
    T[sgn + 2] = varargin_1[3 * i + 2];
    T[i + 12] = 0.0;
    T[sgn + 3] = box[i];
  }
  T[15] = 1.0;
  coder::rigid3d::isTransformationMatrixRigid(T);
  for (i = 0; i < 3; i++) {
    sgn = i << 2;
    K[sgn] = varargin_1[3 * i];
    K[sgn + 1] = varargin_1[3 * i + 1];
    K[sgn + 2] = varargin_1[3 * i + 2];
    K[i + 12] = 0.0;
    K[sgn + 3] = box[i];
  }
  K[15] = 1.0;
  coder::rigid3d::isTransformationMatrixRigid(K);
  for (i = 0; i < 16; i++) {
    b_T[i] = static_cast<float>(T[i]);
  }
  lobj_0.matlabCodegenIsDeleted = true;
  nv.set_size(buoyPT->Location.size(0), 3);
  sgn = buoyPT->Location.size(0) * 3;
  for (i = 0; i < sgn; i++) {
    nv[i] = buoyPT->Location[i];
  }
  sgn = nv.size(0);
  b_loc.set_size(nv.size(0), 3);
  for (j = 0; j < 3; j++) {
    coffset = j * sgn;
    boffset = j * 3;
    for (i = 0; i < sgn; i++) {
      b_loc[coffset + i] =
          (nv[i] * b_T[boffset % 3 + (div_nzp_s32_floor(boffset, 3) << 2)] +
           nv[nv.size(0) + i] * b_T[(boffset + 1) % 3 +
                                    (div_nzp_s32_floor(boffset + 1, 3) << 2)]) +
          nv[(nv.size(0) << 1) + i] *
              b_T[(boffset + 2) % 3 + (div_nzp_s32_floor(boffset + 2, 3) << 2)];
    }
  }
  sgn = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (i = 0; i <= sgn; i++) {
    c_loc[i] = b_loc[i] + b_T[3];
  }
  sgn = c_loc.size(0);
  for (i = 0; i < sgn; i++) {
    b_loc[i] = c_loc[i];
  }
  sgn = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (i = 0; i <= sgn; i++) {
    c_loc[i] = b_loc[i + b_loc.size(0)] + b_T[7];
  }
  sgn = c_loc.size(0);
  for (i = 0; i < sgn; i++) {
    b_loc[i + b_loc.size(0)] = c_loc[i];
  }
  sgn = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (i = 0; i <= sgn; i++) {
    c_loc[i] = b_loc[i + b_loc.size(0) * 2] + b_T[11];
  }
  sgn = c_loc.size(0);
  for (i = 0; i < sgn; i++) {
    b_loc[i + b_loc.size(0) * 2] = c_loc[i];
  }
  nv.set_size(0, 0);
  b_p = (buoyPT->Normal.size(0) == 0);
  b = (buoyPT->Normal.size(1) == 0);
  if ((!b_p) && (!b)) {
    if (coder::vision::internal::isRigidTransform(b_T)) {
      float c_T[9];
      nv.set_size(buoyPT->Normal.size(0), buoyPT->Normal.size(1));
      sgn = buoyPT->Normal.size(0) * buoyPT->Normal.size(1);
      for (i = 0; i < sgn; i++) {
        nv[i] = buoyPT->Normal[i];
      }
      for (i = 0; i < 3; i++) {
        sgn = i << 2;
        c_T[3 * i] = b_T[sgn];
        c_T[3 * i + 1] = b_T[sgn + 1];
        c_T[3 * i + 2] = b_T[sgn + 2];
      }
      b_nv.set_size(nv.size(0), nv.size(1));
      sgn = nv.size(0) * nv.size(1) - 1;
      for (i = 0; i <= sgn; i++) {
        b_nv[i] = nv[i];
      }
      coder::internal::blas::mtimes(b_nv, c_T, nv);
    } else {
      lobj_0.Location.set_size(b_loc.size(0), 3);
      sgn = b_loc.size(0) * 3;
      for (i = 0; i < sgn; i++) {
        lobj_0.Location[i] = b_loc[i];
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
  for (i = 0; i < 4; i++) {
    K23 = T[i];
    K12 = T[i + 4];
    K14 = T[i + 8];
    K13 = T[i + 12];
    for (coffset = 0; coffset < 4; coffset++) {
      sgn = coffset << 2;
      K[i + sgn] = ((K23 * tformICP.AffineTform.T[sgn] +
                     K12 * tformICP.AffineTform.T[sgn + 1]) +
                    K14 * tformICP.AffineTform.T[sgn + 2]) +
                   K13 * tformICP.AffineTform.T[sgn + 3];
    }
  }
  coder::rigid3d::isTransformationMatrixRigid(K);
  coder::rigid3d::isTransformationMatrixRigid(K);
  //  Verify plane confidence
  //  Get Z normal
  box[2] = model.Parameters[2];
  //  Ratio in plane
  sgn = subPT->Location.size(0);
  if (indexOnPlane.size(0) < 1) {
    i = 1;
  } else {
    i = indexOnPlane.size(0);
  }
  *confidence = 100.0 * (1.0 - std::abs(box[2])) *
                (static_cast<double>(i) / static_cast<double>(sgn));
  //  Teturn transformation and confidence
  for (i = 0; i < 3; i++) {
    p[i] = K[(i << 2) + 3];
    varargin_1[3 * i] = K[i];
    varargin_1[3 * i + 1] = K[i + 4];
    varargin_1[3 * i + 2] = K[i + 8];
  }
  double K34;
  K12 = varargin_1[1] + varargin_1[3];
  K13 = varargin_1[2] + varargin_1[6];
  K14 = varargin_1[5] - varargin_1[7];
  K23 = varargin_1[5] + varargin_1[7];
  K24 = varargin_1[6] - varargin_1[2];
  K34 = varargin_1[1] - varargin_1[3];
  K[0] = ((varargin_1[0] - varargin_1[4]) - varargin_1[8]) / 3.0;
  K[4] = K12 / 3.0;
  K[8] = K13 / 3.0;
  K[12] = K14 / 3.0;
  K[1] = K12 / 3.0;
  K[5] = ((varargin_1[4] - varargin_1[0]) - varargin_1[8]) / 3.0;
  K[9] = K23 / 3.0;
  K[13] = K24 / 3.0;
  K[2] = K13 / 3.0;
  K[6] = K23 / 3.0;
  K[10] = ((varargin_1[8] - varargin_1[0]) - varargin_1[4]) / 3.0;
  K[14] = K34 / 3.0;
  K[3] = K14 / 3.0;
  K[7] = K24 / 3.0;
  K[11] = K34 / 3.0;
  K[15] = ((varargin_1[0] + varargin_1[4]) + varargin_1[8]) / 3.0;
  b_p = true;
  for (boffset = 0; boffset < 16; boffset++) {
    if ((!b_p) || (std::isinf(K[boffset]) || std::isnan(K[boffset]))) {
      b_p = false;
    }
  }
  if (!b_p) {
    for (i = 0; i < 16; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
    }
    alpha1[0].re = rtNaN;
    alpha1[1].re = rtNaN;
    alpha1[2].re = rtNaN;
    alpha1[3].re = rtNaN;
  } else {
    int exitg1;
    b_p = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < 4)) {
      i = 0;
      do {
        exitg1 = 0;
        if (i <= j) {
          if (!(K[i + (j << 2)] == K[j + (i << 2)])) {
            b_p = false;
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (b_p) {
      coder::schur(K, T);
      for (i = 0; i < 16; i++) {
        V[i].re = T[i];
        V[i].im = 0.0;
      }
      alpha1[0].re = K[0];
      alpha1[1].re = K[5];
      alpha1[2].re = K[10];
      alpha1[3].re = K[15];
    } else {
      b_p = true;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 4)) {
        i = 0;
        do {
          exitg1 = 0;
          if (i <= j) {
            if (!(K[i + (j << 2)] == -K[j + (i << 2)])) {
              b_p = false;
              exitg1 = 1;
            } else {
              i++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (b_p) {
        coder::schur(K, T);
        i = 1;
        do {
          exitg1 = 0;
          if (i <= 4) {
            bool guard1{false};
            guard1 = false;
            if (i != 4) {
              K23 = K[i + ((i - 1) << 2)];
              if (K23 != 0.0) {
                K12 = std::abs(K23);
                alpha1[i - 1].re = 0.0;
                alpha1[i - 1].im = K12;
                alpha1[i].re = 0.0;
                alpha1[i].im = -K12;
                i += 2;
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
            if (guard1) {
              alpha1[i - 1].re = 0.0;
              alpha1[i - 1].im = 0.0;
              i++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        for (i = 0; i < 16; i++) {
          V[i].re = T[i];
          V[i].im = 0.0;
        }
        j = 1;
        do {
          exitg1 = 0;
          if (j <= 4) {
            if (j != 4) {
              i = (j - 1) << 2;
              if (K[j + i] != 0.0) {
                if (K[j + ((j - 1) << 2)] < 0.0) {
                  sgn = 1;
                } else {
                  sgn = -1;
                }
                K12 = V[i].re;
                coffset = j << 2;
                K13 = static_cast<double>(sgn) * V[coffset].re;
                if (K13 == 0.0) {
                  V[i].re = K12 / 1.4142135623730951;
                  V[i].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i].re = 0.0;
                  V[i].im = K13 / 1.4142135623730951;
                } else {
                  V[i].re = K12 / 1.4142135623730951;
                  V[i].im = K13 / 1.4142135623730951;
                }
                V[coffset].re = V[i].re;
                V[coffset].im = -V[i].im;
                K12 = V[i + 1].re;
                K13 = static_cast<double>(sgn) * V[coffset + 1].re;
                if (K13 == 0.0) {
                  V[i + 1].re = K12 / 1.4142135623730951;
                  V[i + 1].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 1].re = 0.0;
                  V[i + 1].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 1].re = K12 / 1.4142135623730951;
                  V[i + 1].im = K13 / 1.4142135623730951;
                }
                V[coffset + 1].re = V[i + 1].re;
                V[coffset + 1].im = -V[i + 1].im;
                K12 = V[i + 2].re;
                K13 = static_cast<double>(sgn) * V[coffset + 2].re;
                if (K13 == 0.0) {
                  V[i + 2].re = K12 / 1.4142135623730951;
                  V[i + 2].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 2].re = 0.0;
                  V[i + 2].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 2].re = K12 / 1.4142135623730951;
                  V[i + 2].im = K13 / 1.4142135623730951;
                }
                V[coffset + 2].re = V[i + 2].re;
                V[coffset + 2].im = -V[i + 2].im;
                K12 = V[i + 3].re;
                K13 = static_cast<double>(sgn) * V[coffset + 3].re;
                if (K13 == 0.0) {
                  V[i + 3].re = K12 / 1.4142135623730951;
                  V[i + 3].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 3].re = 0.0;
                  V[i + 3].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 3].re = K12 / 1.4142135623730951;
                  V[i + 3].im = K13 / 1.4142135623730951;
                }
                V[coffset + 3].re = V[i + 3].re;
                V[coffset + 3].im = -V[i + 3].im;
                j += 2;
              } else {
                j++;
              }
            } else {
              j++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        creal_T At[16];
        creal_T beta1[4];
        for (i = 0; i < 16; i++) {
          At[i].re = K[i];
          At[i].im = 0.0;
        }
        coder::internal::reflapack::xzggev(At, &sgn, alpha1, beta1, V);
        for (coffset = 0; coffset <= 12; coffset += 4) {
          K24 = 0.0;
          K13 = 3.3121686421112381E-170;
          sgn = coffset + 4;
          for (boffset = coffset + 1; boffset <= sgn; boffset++) {
            K14 = std::abs(V[boffset - 1].re);
            if (K14 > K13) {
              K23 = K13 / K14;
              K24 = K24 * K23 * K23 + 1.0;
              K13 = K14;
            } else {
              K23 = K14 / K13;
              K24 += K23 * K23;
            }
            K14 = std::abs(V[boffset - 1].im);
            if (K14 > K13) {
              K23 = K13 / K14;
              K24 = K24 * K23 * K23 + 1.0;
              K13 = K14;
            } else {
              K23 = K14 / K13;
              K24 += K23 * K23;
            }
          }
          K24 = K13 * std::sqrt(K24);
          for (j = coffset + 1; j <= sgn; j++) {
            K12 = V[j - 1].re;
            K13 = V[j - 1].im;
            if (K13 == 0.0) {
              K14 = K12 / K24;
              K12 = 0.0;
            } else if (K12 == 0.0) {
              K14 = 0.0;
              K12 = K13 / K24;
            } else {
              K14 = K12 / K24;
              K12 = K13 / K24;
            }
            V[j - 1].re = K14;
            V[j - 1].im = K12;
          }
        }
        if (beta1[0].im == 0.0) {
          if (alpha1[0].im == 0.0) {
            K14 = alpha1[0].re / beta1[0].re;
          } else if (alpha1[0].re == 0.0) {
            K14 = 0.0;
          } else {
            K14 = alpha1[0].re / beta1[0].re;
          }
        } else if (beta1[0].re == 0.0) {
          if (alpha1[0].re == 0.0) {
            K14 = alpha1[0].im / beta1[0].im;
          } else if (alpha1[0].im == 0.0) {
            K14 = 0.0;
          } else {
            K14 = alpha1[0].im / beta1[0].im;
          }
        } else {
          K12 = std::abs(beta1[0].re);
          K14 = std::abs(beta1[0].im);
          if (K12 > K14) {
            K12 = beta1[0].im / beta1[0].re;
            K14 = (alpha1[0].re + K12 * alpha1[0].im) /
                  (beta1[0].re + K12 * beta1[0].im);
          } else if (K14 == K12) {
            if (beta1[0].re > 0.0) {
              K23 = 0.5;
            } else {
              K23 = -0.5;
            }
            if (beta1[0].im > 0.0) {
              K14 = 0.5;
            } else {
              K14 = -0.5;
            }
            K14 = (alpha1[0].re * K23 + alpha1[0].im * K14) / K12;
          } else {
            K12 = beta1[0].re / beta1[0].im;
            K14 = (K12 * alpha1[0].re + alpha1[0].im) /
                  (beta1[0].im + K12 * beta1[0].re);
          }
        }
        alpha1[0].re = K14;
        if (beta1[1].im == 0.0) {
          if (alpha1[1].im == 0.0) {
            K14 = alpha1[1].re / beta1[1].re;
          } else if (alpha1[1].re == 0.0) {
            K14 = 0.0;
          } else {
            K14 = alpha1[1].re / beta1[1].re;
          }
        } else if (beta1[1].re == 0.0) {
          if (alpha1[1].re == 0.0) {
            K14 = alpha1[1].im / beta1[1].im;
          } else if (alpha1[1].im == 0.0) {
            K14 = 0.0;
          } else {
            K14 = alpha1[1].im / beta1[1].im;
          }
        } else {
          K12 = std::abs(beta1[1].re);
          K14 = std::abs(beta1[1].im);
          if (K12 > K14) {
            K12 = beta1[1].im / beta1[1].re;
            K14 = (alpha1[1].re + K12 * alpha1[1].im) /
                  (beta1[1].re + K12 * beta1[1].im);
          } else if (K14 == K12) {
            if (beta1[1].re > 0.0) {
              K23 = 0.5;
            } else {
              K23 = -0.5;
            }
            if (beta1[1].im > 0.0) {
              K14 = 0.5;
            } else {
              K14 = -0.5;
            }
            K14 = (alpha1[1].re * K23 + alpha1[1].im * K14) / K12;
          } else {
            K12 = beta1[1].re / beta1[1].im;
            K14 = (K12 * alpha1[1].re + alpha1[1].im) /
                  (beta1[1].im + K12 * beta1[1].re);
          }
        }
        alpha1[1].re = K14;
        if (beta1[2].im == 0.0) {
          if (alpha1[2].im == 0.0) {
            K14 = alpha1[2].re / beta1[2].re;
          } else if (alpha1[2].re == 0.0) {
            K14 = 0.0;
          } else {
            K14 = alpha1[2].re / beta1[2].re;
          }
        } else if (beta1[2].re == 0.0) {
          if (alpha1[2].re == 0.0) {
            K14 = alpha1[2].im / beta1[2].im;
          } else if (alpha1[2].im == 0.0) {
            K14 = 0.0;
          } else {
            K14 = alpha1[2].im / beta1[2].im;
          }
        } else {
          K12 = std::abs(beta1[2].re);
          K14 = std::abs(beta1[2].im);
          if (K12 > K14) {
            K12 = beta1[2].im / beta1[2].re;
            K14 = (alpha1[2].re + K12 * alpha1[2].im) /
                  (beta1[2].re + K12 * beta1[2].im);
          } else if (K14 == K12) {
            if (beta1[2].re > 0.0) {
              K23 = 0.5;
            } else {
              K23 = -0.5;
            }
            if (beta1[2].im > 0.0) {
              K14 = 0.5;
            } else {
              K14 = -0.5;
            }
            K14 = (alpha1[2].re * K23 + alpha1[2].im * K14) / K12;
          } else {
            K12 = beta1[2].re / beta1[2].im;
            K14 = (K12 * alpha1[2].re + alpha1[2].im) /
                  (beta1[2].im + K12 * beta1[2].re);
          }
        }
        alpha1[2].re = K14;
        if (beta1[3].im == 0.0) {
          if (alpha1[3].im == 0.0) {
            K14 = alpha1[3].re / beta1[3].re;
          } else if (alpha1[3].re == 0.0) {
            K14 = 0.0;
          } else {
            K14 = alpha1[3].re / beta1[3].re;
          }
        } else if (beta1[3].re == 0.0) {
          if (alpha1[3].re == 0.0) {
            K14 = alpha1[3].im / beta1[3].im;
          } else if (alpha1[3].im == 0.0) {
            K14 = 0.0;
          } else {
            K14 = alpha1[3].im / beta1[3].im;
          }
        } else {
          K12 = std::abs(beta1[3].re);
          K14 = std::abs(beta1[3].im);
          if (K12 > K14) {
            K12 = beta1[3].im / beta1[3].re;
            K14 = (alpha1[3].re + K12 * alpha1[3].im) /
                  (beta1[3].re + K12 * beta1[3].im);
          } else if (K14 == K12) {
            if (beta1[3].re > 0.0) {
              K23 = 0.5;
            } else {
              K23 = -0.5;
            }
            if (beta1[3].im > 0.0) {
              K14 = 0.5;
            } else {
              K14 = -0.5;
            }
            K14 = (alpha1[3].re * K23 + alpha1[3].im * K14) / K12;
          } else {
            K12 = beta1[3].re / beta1[3].im;
            K14 = (K12 * alpha1[3].re + alpha1[3].im) /
                  (beta1[3].im + K12 * beta1[3].re);
          }
        }
        alpha1[3].re = K14;
      }
    }
  }
  qApprox[0] = alpha1[0].re;
  qApprox[1] = alpha1[1].re;
  qApprox[2] = alpha1[2].re;
  qApprox[3] = alpha1[3].re;
  if (!std::isnan(alpha1[0].re)) {
    sgn = 1;
  } else {
    sgn = 0;
    boffset = 2;
    exitg2 = false;
    while ((!exitg2) && (boffset < 5)) {
      if (!std::isnan(qApprox[boffset - 1])) {
        sgn = boffset;
        exitg2 = true;
      } else {
        boffset++;
      }
    }
  }
  if (sgn == 0) {
    coffset = 0;
  } else {
    K12 = qApprox[sgn - 1];
    coffset = sgn - 1;
    i = sgn + 1;
    for (boffset = i; boffset < 5; boffset++) {
      K23 = qApprox[boffset - 1];
      if (K12 < K23) {
        K12 = K23;
        coffset = boffset - 1;
      }
    }
  }
  sgn = coffset << 2;
  qApprox[0] = V[sgn + 3].re;
  qApprox[1] = V[sgn].re;
  qApprox[2] = V[sgn + 1].re;
  qApprox[3] = V[sgn + 2].re;
  if (qApprox[0] < 0.0) {
    qApprox[0] = -qApprox[0];
    qApprox[1] = -qApprox[1];
    qApprox[2] = -qApprox[2];
    qApprox[3] = -qApprox[3];
  }
  // =================================================================
  //  Fonction qui calcule l'angle entre 2 quaternion
  q[0] = qApprox[0];
  q[1] = qApprox[1];
  q[2] = qApprox[2];
  q[3] = qApprox[3];
  K13 = 3.3121686421112381E-170;
  K14 = std::abs((qApprox[0] * auvQuat[1] + auvQuat[0] * -qApprox[1]) +
                 (-qApprox[2] * auvQuat[3] - auvQuat[2] * -qApprox[3]));
  if (K14 > 3.3121686421112381E-170) {
    K12 = 1.0;
    K13 = K14;
  } else {
    K23 = K14 / 3.3121686421112381E-170;
    K12 = K23 * K23;
  }
  K14 = std::abs((qApprox[0] * auvQuat[2] + auvQuat[0] * -qApprox[2]) +
                 (auvQuat[1] * -qApprox[3] - -qApprox[1] * auvQuat[3]));
  if (K14 > K13) {
    K23 = K13 / K14;
    K12 = K12 * K23 * K23 + 1.0;
    K13 = K14;
  } else {
    K23 = K14 / K13;
    K12 += K23 * K23;
  }
  K14 = std::abs((qApprox[0] * auvQuat[3] + auvQuat[0] * -qApprox[3]) +
                 (-qApprox[1] * auvQuat[2] - auvQuat[1] * -qApprox[2]));
  if (K14 > K13) {
    K23 = K13 / K14;
    K12 = K12 * K23 * K23 + 1.0;
    K13 = K14;
  } else {
    K23 = K14 / K13;
    K12 += K23 * K23;
  }
  K12 = K13 * std::sqrt(K12);
  K12 = std::abs(2.0 * rt_atan2d_snf(K12, ((qApprox[0] * auvQuat[0] -
                                            -qApprox[1] * auvQuat[1]) -
                                           -qApprox[2] * auvQuat[2]) -
                                              -qApprox[3] * auvQuat[3]));
  if (std::fmin(6.2831853071795862 - K12, K12) > 1.5707963267948966) {
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
  K12 = box[1] * box[2];
  if ((K12 < param.minArea) || (K12 > param.maxArea)) {
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
  int i;
  int last;
  int loop_ub;
  int loop_ub_tmp;
  int unnamed_idx_0;
  clusterPT.matlabCodegenIsDeleted = true;
  b_clusterPT.matlabCodegenIsDeleted = true;
  coder::pcsegdist(filteredPT, param.clusterDist, r, &area);
  PTlabels.set_size(r.size(0), 1);
  last = r.size(0);
  for (i = 0; i < last; i++) {
    PTlabels[i] = r[i];
  }
  loop_ub_tmp = static_cast<int>(area);
  goodCluster.set_size(1, loop_ub_tmp);
  sonia_common_ObstacleInfoStruct(&feature[1]);
  //  Get the good clusters.
  if (static_cast<int>(area) - 1 >= 0) {
    unnamed_idx_0 = r.size(0);
    loop_ub = r.size(0);
  }
  for (int b_i{0}; b_i < loop_ub_tmp; b_i++) {
    bool b;
    //  Extract pointCloud
    plane.matlabCodegenIsDeleted = true;
    c_clusterPT.matlabCodegenIsDeleted = true;
    r1.set_size(unnamed_idx_0, 1);
    for (i = 0; i < loop_ub; i++) {
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
    last = b_r.size(0) * b_r.size(1);
    for (i = 0; i < last; i++) {
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
  //  One or more clusters found.
  area = coder::combineVectorElements(goodCluster);
  if (area != 0.0) {
    last = static_cast<int>(area);
    distances.set_size(1, last);
    for (i = 0; i < last; i++) {
      distances[i] = 0.0;
    }
    if (static_cast<int>(area) - 1 >= 0) {
      b_loop_ub = goodCluster.size(1);
      unnamed_idx_0 = r.size(0);
      c_loop_ub = r.size(0);
    }
    if (last - 1 >= 0) {
      p[1] = auvPose[0];
      p[3] = auvPose[1];
      p[5] = auvPose[2];
    }
    for (int b_i{0}; b_i < last; b_i++) {
      b_goodCluster.set_size(1, goodCluster.size(1));
      for (i = 0; i < b_loop_ub; i++) {
        b_goodCluster[i] = (goodCluster[i] == 1.0);
      }
      coder::eml_find(b_goodCluster, b_index);
      loop_ub = b_index[b_i];
      r1.set_size(unnamed_idx_0, 1);
      for (i = 0; i < c_loop_ub; i++) {
        r1[i] = (static_cast<double>(r[i]) == loop_ub);
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
        unnamed_idx_0 = 2;
        exitg1 = false;
        while ((!exitg1) && (unnamed_idx_0 <= last)) {
          if (!std::isnan(distances[unnamed_idx_0 - 1])) {
            loop_ub = unnamed_idx_0;
            exitg1 = true;
          } else {
            unnamed_idx_0++;
          }
        }
      }
      if (loop_ub == 0) {
        loop_ub = 1;
      } else {
        area = distances[loop_ub - 1];
        i = loop_ub + 1;
        for (unnamed_idx_0 = i; unnamed_idx_0 <= last; unnamed_idx_0++) {
          offset_idx_2 = distances[unnamed_idx_0 - 1];
          if (area > offset_idx_2) {
            area = offset_idx_2;
            loop_ub = unnamed_idx_0;
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
    last = goodCluster.size(1);
    for (i = 0; i < last; i++) {
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
    double offset_idx_0;
    double offset_idx_1;
    feature[1].Confidence = static_cast<float>(area);
    box[1] = param.gap / 2.0;
    // =================================================================
    //  Fonction qui tourne un vecteur selon un quaternion.
    //  quaternion partie scalaire
    //  quaternion partie vectoriel
    //  QuatRotate n'est pas compilable
    area = 2.0 * ((q[1] * 0.0 + box[1] * q[2]) + q[3] * 0.0);
    offset_idx_2 = q[0] * q[0] - ((q[1] * q[1] + q[2] * q[2]) + q[3] * q[3]);
    a = 2.0 * q[0];
    offset_idx_0 =
        (area * q[1] + offset_idx_2 * 0.0) + a * (q[2] * 0.0 - box[1] * q[3]);
    offset_idx_1 =
        (area * q[2] + offset_idx_2 * box[1]) + a * (q[3] * 0.0 - q[1] * 0.0);
    offset_idx_2 =
        (area * q[3] + offset_idx_2 * 0.0) + a * (q[1] * box[1] - q[2] * 0.0);
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
    //  No clusters found.
  } else {
    feature[1].IsValid = false;
    feature[1].Name.set_size(1, 5);
    for (i = 0; i < 5; i++) {
      feature[1].Name[i] = b_cv[i];
    }
    feature[1].Confidence = 0.0F;
    feature[0] = feature[1];
  }
}

//
// File trailer for Buoys.cpp
//
// [EOF]
//
