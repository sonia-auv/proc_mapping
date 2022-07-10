//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Buoys.cpp
//
// Code generation for function 'Buoys'
//

// Include files
#include "Buoys.h"
#include "Kdtree.h"
#include "affine3d.h"
#include "pcfitplane.h"
#include "pcregistericp.h"
#include "pctransform.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "proc_mapping_data.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "quat2rotm.h"
#include "quatUtilities.h"
#include "rigid3d.h"
#include "rigid3dImpl.h"
#include "rt_nonfinite.h"
#include "schur.h"
#include "xzggev.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <string.h>

// Function Definitions
void Buoys::getBuoyPose(coder::pointCloud *subPT, const double auvQuat[4],
                        double p[3], double q[4]) const
{
  static coder::b_pointCloud buoyTformed;
  coder::images::internal::rigid3dImpl b_r;
  coder::planeModel model;
  coder::planeModel *b_model;
  coder::pointCloud plane;
  coder::pointCloud *ptCloudOut;
  coder::rigid3d tformRansac;
  coder::vision::internal::codegen::Kdtree lobj_3;
  coder::vision::internal::codegen::Kdtree lobj_4;
  coder::array<double, 2U> loc;
  coder::array<double, 2U> meanError;
  coder::array<double, 2U> r;
  coder::array<double, 1U> a__2;
  coder::array<double, 1U> indexOnPlane;
  coder::array<unsigned char, 2U> c;
  creal_T V[16];
  creal_T alpha1[4];
  double K[16];
  double T[16];
  double b_varargin_1[16];
  double varargin_1[9];
  double qApprox[4];
  double pApprox[3];
  double K13;
  double K14;
  double K23;
  double K24;
  double qnrm;
  int coltop;
  int i;
  int k;
  int sgn;
  bool b_p;
  bool exitg2;
  plane.matlabCodegenIsDeleted = true;
  buoyTformed.matlabCodegenIsDeleted = true;
  //  Apply ransac
  coder::pcfitplane(subPT, param.planeTol, &model, &b_model, indexOnPlane, a__2,
                    meanError);
  subPT->subsetImpl(indexOnPlane, loc, c, meanError, a__2, r);
  ptCloudOut = plane.init(loc, c, meanError, a__2, &lobj_4);
  ptCloudOut->RangeData.set_size(r.size(0), r.size(1));
  sgn = r.size(0) * r.size(1);
  for (i = 0; i < sgn; i++) {
    ptCloudOut->RangeData[i] = r[i];
  }
  //  Get ransac plane pose approximation
  quatUtilities::getOrientedPointOnPlanarFace(&model, subPT, pApprox, qApprox);
  //  Transform the buoy on the plane.
  qnrm = ((qApprox[0] * qApprox[0] + qApprox[1] * qApprox[1]) +
          qApprox[2] * qApprox[2]) +
         qApprox[3] * qApprox[3];
  qApprox[0] /= qnrm;
  qApprox[1] = -qApprox[1] / qnrm;
  qApprox[2] = -qApprox[2] / qnrm;
  qApprox[3] = -qApprox[3] / qnrm;
  coder::quat2rotm(qApprox, varargin_1);
  for (i = 0; i < 3; i++) {
    sgn = i << 2;
    T[sgn] = varargin_1[3 * i];
    T[sgn + 1] = varargin_1[3 * i + 1];
    T[sgn + 2] = varargin_1[3 * i + 2];
    T[i + 12] = 0.0;
    T[sgn + 3] = pApprox[i];
  }
  T[15] = 1.0;
  coder::rigid3d::isTransformationMatrixRigid(T);
  std::copy(&T[0], &T[16], &tformRansac.AffineTform.T[0]);
  for (i = 0; i < 3; i++) {
    sgn = i << 2;
    b_varargin_1[sgn] = varargin_1[3 * i];
    b_varargin_1[sgn + 1] = varargin_1[3 * i + 1];
    b_varargin_1[sgn + 2] = varargin_1[3 * i + 2];
    b_varargin_1[i + 12] = 0.0;
    b_varargin_1[sgn + 3] = pApprox[i];
  }
  b_varargin_1[15] = 1.0;
  coder::rigid3d::isTransformationMatrixRigid(b_varargin_1);
  tformRansac.Data.set_size(1, 1);
  tformRansac.Data[0] = b_r;
  coder::pctransform(buoyPT, &tformRansac, &lobj_3, &buoyTformed);
  //  Apply icp.
  coder::pcregistericp(&buoyTformed, &plane, param.icpInlierRatio,
                       &tformRansac);
  //  Get buoys transformation.
  for (i = 0; i < 4; i++) {
    K23 = T[i];
    qnrm = T[i + 4];
    K14 = T[i + 8];
    K13 = T[i + 12];
    for (coltop = 0; coltop < 4; coltop++) {
      sgn = coltop << 2;
      b_varargin_1[i + sgn] = ((K23 * tformRansac.AffineTform.T[sgn] +
                                qnrm * tformRansac.AffineTform.T[sgn + 1]) +
                               K14 * tformRansac.AffineTform.T[sgn + 2]) +
                              K13 * tformRansac.AffineTform.T[sgn + 3];
    }
  }
  coder::rigid3d::isTransformationMatrixRigid(b_varargin_1);
  coder::rigid3d::isTransformationMatrixRigid(b_varargin_1);
  //  return transform
  for (i = 0; i < 3; i++) {
    p[i] = b_varargin_1[(i << 2) + 3];
    varargin_1[3 * i] = b_varargin_1[i];
    varargin_1[3 * i + 1] = b_varargin_1[i + 4];
    varargin_1[3 * i + 2] = b_varargin_1[i + 8];
  }
  double K34;
  qnrm = varargin_1[1] + varargin_1[3];
  K13 = varargin_1[2] + varargin_1[6];
  K14 = varargin_1[5] - varargin_1[7];
  K23 = varargin_1[5] + varargin_1[7];
  K24 = varargin_1[6] - varargin_1[2];
  K34 = varargin_1[1] - varargin_1[3];
  K[0] = ((varargin_1[0] - varargin_1[4]) - varargin_1[8]) / 3.0;
  K[4] = qnrm / 3.0;
  K[8] = K13 / 3.0;
  K[12] = K14 / 3.0;
  K[1] = qnrm / 3.0;
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
  for (k = 0; k < 16; k++) {
    if ((!b_p) || (std::isinf(K[k]) || std::isnan(K[k]))) {
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
    k = 0;
    exitg2 = false;
    while ((!exitg2) && (k < 4)) {
      sgn = 0;
      do {
        exitg1 = 0;
        if (sgn <= k) {
          if (!(K[sgn + (k << 2)] == K[k + (sgn << 2)])) {
            b_p = false;
            exitg1 = 1;
          } else {
            sgn++;
          }
        } else {
          k++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (b_p) {
      coder::schur(K, b_varargin_1, T);
      for (i = 0; i < 16; i++) {
        V[i].re = b_varargin_1[i];
        V[i].im = 0.0;
      }
      alpha1[0].re = T[0];
      alpha1[1].re = T[5];
      alpha1[2].re = T[10];
      alpha1[3].re = T[15];
    } else {
      b_p = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 4)) {
        sgn = 0;
        do {
          exitg1 = 0;
          if (sgn <= k) {
            if (!(K[sgn + (k << 2)] == -K[k + (sgn << 2)])) {
              b_p = false;
              exitg1 = 1;
            } else {
              sgn++;
            }
          } else {
            k++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (b_p) {
        coder::schur(K, b_varargin_1, T);
        sgn = 1;
        do {
          exitg1 = 0;
          if (sgn <= 4) {
            bool guard1{false};
            guard1 = false;
            if (sgn != 4) {
              K23 = T[sgn + ((sgn - 1) << 2)];
              if (K23 != 0.0) {
                qnrm = std::abs(K23);
                alpha1[sgn - 1].re = 0.0;
                alpha1[sgn - 1].im = qnrm;
                alpha1[sgn].re = 0.0;
                alpha1[sgn].im = -qnrm;
                sgn += 2;
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
            if (guard1) {
              alpha1[sgn - 1].re = 0.0;
              alpha1[sgn - 1].im = 0.0;
              sgn++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        for (i = 0; i < 16; i++) {
          V[i].re = b_varargin_1[i];
          V[i].im = 0.0;
        }
        k = 1;
        do {
          exitg1 = 0;
          if (k <= 4) {
            if (k != 4) {
              i = (k - 1) << 2;
              if (T[k + i] != 0.0) {
                if (T[k + ((k - 1) << 2)] < 0.0) {
                  sgn = 1;
                } else {
                  sgn = -1;
                }
                qnrm = V[i].re;
                coltop = k << 2;
                K13 = static_cast<double>(sgn) * V[coltop].re;
                if (K13 == 0.0) {
                  V[i].re = qnrm / 1.4142135623730951;
                  V[i].im = 0.0;
                } else if (qnrm == 0.0) {
                  V[i].re = 0.0;
                  V[i].im = K13 / 1.4142135623730951;
                } else {
                  V[i].re = qnrm / 1.4142135623730951;
                  V[i].im = K13 / 1.4142135623730951;
                }
                V[coltop].re = V[i].re;
                V[coltop].im = -V[i].im;
                qnrm = V[i + 1].re;
                K13 = static_cast<double>(sgn) * V[coltop + 1].re;
                if (K13 == 0.0) {
                  V[i + 1].re = qnrm / 1.4142135623730951;
                  V[i + 1].im = 0.0;
                } else if (qnrm == 0.0) {
                  V[i + 1].re = 0.0;
                  V[i + 1].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 1].re = qnrm / 1.4142135623730951;
                  V[i + 1].im = K13 / 1.4142135623730951;
                }
                V[coltop + 1].re = V[i + 1].re;
                V[coltop + 1].im = -V[i + 1].im;
                qnrm = V[i + 2].re;
                K13 = static_cast<double>(sgn) * V[coltop + 2].re;
                if (K13 == 0.0) {
                  V[i + 2].re = qnrm / 1.4142135623730951;
                  V[i + 2].im = 0.0;
                } else if (qnrm == 0.0) {
                  V[i + 2].re = 0.0;
                  V[i + 2].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 2].re = qnrm / 1.4142135623730951;
                  V[i + 2].im = K13 / 1.4142135623730951;
                }
                V[coltop + 2].re = V[i + 2].re;
                V[coltop + 2].im = -V[i + 2].im;
                qnrm = V[i + 3].re;
                K13 = static_cast<double>(sgn) * V[coltop + 3].re;
                if (K13 == 0.0) {
                  V[i + 3].re = qnrm / 1.4142135623730951;
                  V[i + 3].im = 0.0;
                } else if (qnrm == 0.0) {
                  V[i + 3].re = 0.0;
                  V[i + 3].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 3].re = qnrm / 1.4142135623730951;
                  V[i + 3].im = K13 / 1.4142135623730951;
                }
                V[coltop + 3].re = V[i + 3].re;
                V[coltop + 3].im = -V[i + 3].im;
                k += 2;
              } else {
                k++;
              }
            } else {
              k++;
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
        for (coltop = 0; coltop <= 12; coltop += 4) {
          K24 = 0.0;
          K13 = 3.3121686421112381E-170;
          sgn = coltop + 4;
          for (k = coltop + 1; k <= sgn; k++) {
            K14 = std::abs(V[k - 1].re);
            if (K14 > K13) {
              K23 = K13 / K14;
              K24 = K24 * K23 * K23 + 1.0;
              K13 = K14;
            } else {
              K23 = K14 / K13;
              K24 += K23 * K23;
            }
            K14 = std::abs(V[k - 1].im);
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
          for (k = coltop + 1; k <= sgn; k++) {
            qnrm = V[k - 1].re;
            K13 = V[k - 1].im;
            if (K13 == 0.0) {
              K14 = qnrm / K24;
              qnrm = 0.0;
            } else if (qnrm == 0.0) {
              K14 = 0.0;
              qnrm = K13 / K24;
            } else {
              K14 = qnrm / K24;
              qnrm = K13 / K24;
            }
            V[k - 1].re = K14;
            V[k - 1].im = qnrm;
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
          qnrm = std::abs(beta1[0].re);
          K14 = std::abs(beta1[0].im);
          if (qnrm > K14) {
            qnrm = beta1[0].im / beta1[0].re;
            K14 = (alpha1[0].re + qnrm * alpha1[0].im) /
                  (beta1[0].re + qnrm * beta1[0].im);
          } else if (K14 == qnrm) {
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
            K14 = (alpha1[0].re * K23 + alpha1[0].im * K14) / qnrm;
          } else {
            qnrm = beta1[0].re / beta1[0].im;
            K14 = (qnrm * alpha1[0].re + alpha1[0].im) /
                  (beta1[0].im + qnrm * beta1[0].re);
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
          qnrm = std::abs(beta1[1].re);
          K14 = std::abs(beta1[1].im);
          if (qnrm > K14) {
            qnrm = beta1[1].im / beta1[1].re;
            K14 = (alpha1[1].re + qnrm * alpha1[1].im) /
                  (beta1[1].re + qnrm * beta1[1].im);
          } else if (K14 == qnrm) {
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
            K14 = (alpha1[1].re * K23 + alpha1[1].im * K14) / qnrm;
          } else {
            qnrm = beta1[1].re / beta1[1].im;
            K14 = (qnrm * alpha1[1].re + alpha1[1].im) /
                  (beta1[1].im + qnrm * beta1[1].re);
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
          qnrm = std::abs(beta1[2].re);
          K14 = std::abs(beta1[2].im);
          if (qnrm > K14) {
            qnrm = beta1[2].im / beta1[2].re;
            K14 = (alpha1[2].re + qnrm * alpha1[2].im) /
                  (beta1[2].re + qnrm * beta1[2].im);
          } else if (K14 == qnrm) {
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
            K14 = (alpha1[2].re * K23 + alpha1[2].im * K14) / qnrm;
          } else {
            qnrm = beta1[2].re / beta1[2].im;
            K14 = (qnrm * alpha1[2].re + alpha1[2].im) /
                  (beta1[2].im + qnrm * beta1[2].re);
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
          qnrm = std::abs(beta1[3].re);
          K14 = std::abs(beta1[3].im);
          if (qnrm > K14) {
            qnrm = beta1[3].im / beta1[3].re;
            K14 = (alpha1[3].re + qnrm * alpha1[3].im) /
                  (beta1[3].re + qnrm * beta1[3].im);
          } else if (K14 == qnrm) {
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
            K14 = (alpha1[3].re * K23 + alpha1[3].im * K14) / qnrm;
          } else {
            qnrm = beta1[3].re / beta1[3].im;
            K14 = (qnrm * alpha1[3].re + alpha1[3].im) /
                  (beta1[3].im + qnrm * beta1[3].re);
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
    k = 2;
    exitg2 = false;
    while ((!exitg2) && (k < 5)) {
      if (!std::isnan(qApprox[k - 1])) {
        sgn = k;
        exitg2 = true;
      } else {
        k++;
      }
    }
  }
  if (sgn == 0) {
    coltop = 0;
  } else {
    qnrm = qApprox[sgn - 1];
    coltop = sgn - 1;
    i = sgn + 1;
    for (k = i; k < 5; k++) {
      K23 = qApprox[k - 1];
      if (qnrm < K23) {
        qnrm = K23;
        coltop = k - 1;
      }
    }
  }
  sgn = coltop << 2;
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
    qnrm = 1.0;
    K13 = K14;
  } else {
    K23 = K14 / 3.3121686421112381E-170;
    qnrm = K23 * K23;
  }
  K14 = std::abs((qApprox[0] * auvQuat[2] + auvQuat[0] * -qApprox[2]) +
                 (auvQuat[1] * -qApprox[3] - -qApprox[1] * auvQuat[3]));
  if (K14 > K13) {
    K23 = K13 / K14;
    qnrm = qnrm * K23 * K23 + 1.0;
    K13 = K14;
  } else {
    K23 = K14 / K13;
    qnrm += K23 * K23;
  }
  K14 = std::abs((qApprox[0] * auvQuat[3] + auvQuat[0] * -qApprox[3]) +
                 (-qApprox[1] * auvQuat[2] - auvQuat[1] * -qApprox[2]));
  if (K14 > K13) {
    K23 = K13 / K14;
    qnrm = qnrm * K23 * K23 + 1.0;
    K13 = K14;
  } else {
    K23 = K14 / K13;
    qnrm += K23 * K23;
  }
  qnrm = K13 * std::sqrt(qnrm);
  if (std::abs(2.0 *
               rt_atan2d_snf(
                   qnrm, ((qApprox[0] * auvQuat[0] - -qApprox[1] * auvQuat[1]) -
                          -qApprox[2] * auvQuat[2]) -
                             -qApprox[3] * auvQuat[3])) > 1.5707963267948966) {
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
}

coder::vision::internal::codegen::Kdtree binary_expand_op(
    coder::pointCloud *in1, const coder::array<unsigned int, 1U> &in2,
    const coder::array<int, 2U> &in3,
    coder::vision::internal::codegen::Kdtree in4, coder::pointCloud *in5)
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
  in1->b_select(b_in2, &in4, in5);
  return in4;
}

// End of code generation (Buoys.cpp)
