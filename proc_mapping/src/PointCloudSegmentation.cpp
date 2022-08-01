//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PointCloudSegmentation.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "PointCloudSegmentation.h"
#include "Kdtree.h"
#include "minOrMax.h"
#include "pointCloud.h"
#include "proc_mapping_rtwutil.h"
#include "quat2rotm.h"
#include "rigid3d.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "PCANormalCore_api.hpp"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <math.h>
#include <string.h>

// Function Definitions
//
// Arguments    : const double q[4]
//                const coder::pointCloud *pc
//                double box[3]
// Return Type  : void
//
void PointCloudSegmentation::objectAllignBoudingBox(const double q[4],
                                                    const coder::pointCloud *pc,
                                                    double box[3])
{
  coder::h_pointCloud lobj_0;
  coder::pointCloud pct;
  coder::vision::internal::codegen::Kdtree lobj_1;
  coder::vision::internal::codegen::b_Kdtree *b_this;
  coder::array<double, 2U> b_lobj_0;
  coder::array<double, 2U> c_loc;
  coder::array<double, 2U> loc;
  coder::array<double, 2U> r;
  coder::array<double, 2U> tempNormals;
  coder::array<double, 1U> b_loc;
  coder::array<unsigned int, 2U> indices;
  coder::array<unsigned int, 1U> valid;
  double T[16];
  double b_tRot[16];
  double tRot[9];
  int boffset;
  int coffset;
  int exponent;
  int i;
  int jA;
  pct.matlabCodegenIsDeleted = true;
  coder::quat2rotm(q, tRot);
  for (i = 0; i < 3; i++) {
    jA = i << 2;
    T[jA] = tRot[3 * i];
    T[jA + 1] = tRot[3 * i + 1];
    T[jA + 2] = tRot[3 * i + 2];
    T[i + 12] = 0.0;
  }
  T[3] = 0.0;
  T[7] = 0.0;
  T[11] = 0.0;
  T[15] = 1.0;
  coder::rigid3d::isTransformationMatrixRigid(T);
  for (i = 0; i < 3; i++) {
    jA = i << 2;
    b_tRot[jA] = tRot[3 * i];
    b_tRot[jA + 1] = tRot[3 * i + 1];
    b_tRot[jA + 2] = tRot[3 * i + 2];
    b_tRot[i + 12] = 0.0;
  }
  b_tRot[3] = 0.0;
  b_tRot[7] = 0.0;
  b_tRot[11] = 0.0;
  b_tRot[15] = 1.0;
  coder::rigid3d::isTransformationMatrixRigid(b_tRot);
  lobj_0.matlabCodegenIsDeleted = true;
  jA = pc->Location.size(0);
  loc.set_size(pc->Location.size(0), 3);
  for (int j{0}; j < 3; j++) {
    coffset = j * jA;
    boffset = j * 3;
    for (int b_i{0}; b_i < jA; b_i++) {
      loc[coffset + b_i] =
          (pc->Location[b_i] *
               T[boffset % 3 + (div_nzp_s32_floor(boffset, 3) << 2)] +
           pc->Location[pc->Location.size(0) + b_i] *
               T[(boffset + 1) % 3 +
                 (div_nzp_s32_floor(boffset + 1, 3) << 2)]) +
          pc->Location[(pc->Location.size(0) << 1) + b_i] *
              T[(boffset + 2) % 3 + (div_nzp_s32_floor(boffset + 2, 3) << 2)];
    }
  }
  jA = loc.size(0) - 1;
  b_loc.set_size(loc.size(0));
  for (i = 0; i <= jA; i++) {
    b_loc[i] = loc[i];
  }
  jA = b_loc.size(0);
  for (i = 0; i < jA; i++) {
    loc[i] = b_loc[i];
  }
  jA = loc.size(0) - 1;
  b_loc.set_size(loc.size(0));
  for (i = 0; i <= jA; i++) {
    b_loc[i] = loc[i + loc.size(0)];
  }
  jA = b_loc.size(0);
  for (i = 0; i < jA; i++) {
    loc[i + loc.size(0)] = b_loc[i];
  }
  jA = loc.size(0) - 1;
  b_loc.set_size(loc.size(0));
  for (i = 0; i <= jA; i++) {
    b_loc[i] = loc[i + loc.size(0) * 2];
  }
  jA = b_loc.size(0);
  for (i = 0; i < jA; i++) {
    loc[i + loc.size(0) * 2] = b_loc[i];
  }
  tempNormals.set_size(0, 0);
  if ((pc->Normal.size(0) != 0) && (pc->Normal.size(1) != 0)) {
    double singularValues[3];
    double absx;
    double s;
    bool guard1{false};
    for (i = 0; i < 3; i++) {
      jA = i << 2;
      tRot[3 * i] = T[jA];
      tRot[3 * i + 1] = T[jA + 1];
      tRot[3 * i + 2] = T[jA + 2];
    }
    coder::svd(tRot, singularValues);
    s = coder::internal::maximum(singularValues);
    absx = std::abs(s);
    if ((!std::isinf(absx)) && (!std::isnan(absx))) {
      if (absx <= 2.2250738585072014E-308) {
        absx = 4.94065645841247E-324;
      } else {
        frexp(absx, &exponent);
        absx = std::ldexp(1.0, exponent - 53);
      }
    } else {
      absx = rtNaN;
    }
    guard1 = false;
    if (s - coder::internal::minimum(singularValues) < 100.0 * absx) {
      int jp1j;
      signed char ipiv[4];
      bool isodd;
      std::copy(&T[0], &T[16], &b_tRot[0]);
      ipiv[0] = 1;
      ipiv[1] = 2;
      ipiv[2] = 3;
      for (int j{0}; j < 3; j++) {
        int b_tmp;
        boffset = 2 - j;
        b_tmp = j * 5;
        jp1j = b_tmp + 2;
        jA = 4 - j;
        exponent = 0;
        absx = std::abs(b_tRot[b_tmp]);
        for (int k{2}; k <= jA; k++) {
          s = std::abs(b_tRot[(b_tmp + k) - 1]);
          if (s > absx) {
            exponent = k - 1;
            absx = s;
          }
        }
        if (b_tRot[b_tmp + exponent] != 0.0) {
          if (exponent != 0) {
            jA = j + exponent;
            ipiv[j] = static_cast<signed char>(jA + 1);
            absx = b_tRot[j];
            b_tRot[j] = b_tRot[jA];
            b_tRot[jA] = absx;
            absx = b_tRot[j + 4];
            b_tRot[j + 4] = b_tRot[jA + 4];
            b_tRot[jA + 4] = absx;
            absx = b_tRot[j + 8];
            b_tRot[j + 8] = b_tRot[jA + 8];
            b_tRot[jA + 8] = absx;
            absx = b_tRot[j + 12];
            b_tRot[j + 12] = b_tRot[jA + 12];
            b_tRot[jA + 12] = absx;
          }
          i = (b_tmp - j) + 4;
          for (int b_i{jp1j}; b_i <= i; b_i++) {
            b_tRot[b_i - 1] /= b_tRot[b_tmp];
          }
        }
        jA = b_tmp;
        for (jp1j = 0; jp1j <= boffset; jp1j++) {
          absx = b_tRot[(b_tmp + (jp1j << 2)) + 4];
          if (absx != 0.0) {
            i = jA + 6;
            exponent = (jA - j) + 8;
            for (coffset = i; coffset <= exponent; coffset++) {
              b_tRot[coffset - 1] +=
                  b_tRot[((b_tmp + coffset) - jA) - 5] * -absx;
            }
          }
          jA += 4;
        }
      }
      isodd = (ipiv[0] > 1);
      if (ipiv[1] > 2) {
        isodd = !isodd;
      }
      absx = b_tRot[0] * b_tRot[5] * b_tRot[10] * b_tRot[15];
      if (ipiv[2] > 3) {
        isodd = !isodd;
      }
      if (isodd) {
        absx = -absx;
      }
      if (std::abs(absx - 1.0) < 2.2204460492503131E-14) {
        exponent = pc->Normal.size(0);
        jA = pc->Normal.size(1);
        c_loc.set_size(pc->Normal.size(0), 3);
        for (int j{0}; j < 3; j++) {
          coffset = j * exponent;
          boffset = j * 3;
          for (int b_i{0}; b_i < exponent; b_i++) {
            c_loc[coffset + b_i] = 0.0;
          }
          for (int k{0}; k < jA; k++) {
            jp1j = k * pc->Normal.size(0);
            i = boffset + k;
            absx = T[i % 3 + (div_nzp_s32_floor(i, 3) << 2)];
            for (int b_i{0}; b_i < exponent; b_i++) {
              i = coffset + b_i;
              c_loc[i] = c_loc[i] + pc->Normal[jp1j + b_i] * absx;
            }
          }
        }
        tempNormals.set_size(c_loc.size(0), 3);
        jA = c_loc.size(0) * 3;
        for (i = 0; i < jA; i++) {
          tempNormals[i] = c_loc[i];
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      lobj_0.Location.set_size(loc.size(0), 3);
      jA = loc.size(0) * 3;
      for (i = 0; i < jA; i++) {
        lobj_0.Location[i] = loc[i];
      }
      lobj_0.Color.set_size(0, 0);
      lobj_0.Normal.set_size(0, 0);
      lobj_0.Intensity.set_size(0, 0);
      lobj_0.b_Kdtree = lobj_0._pobj0.init();
      lobj_0.matlabCodegenIsDeleted = false;
      exponent = lobj_0.Location.size(0);
      jA = lobj_0.Location.size(0);
      if (jA <= 2) {
        unsigned int unnamed_idx_0;
        unnamed_idx_0 = static_cast<unsigned int>(lobj_0.Location.size(0));
        tempNormals.set_size(static_cast<int>(unnamed_idx_0), 3);
        jA = static_cast<int>(unnamed_idx_0) * 3;
        for (i = 0; i < jA; i++) {
          tempNormals[i] = rtNaN;
        }
      } else {
        c_loc.set_size(lobj_0.Location.size(0), 3);
        jA = lobj_0.Location.size(0) * 3;
        for (i = 0; i < jA; i++) {
          c_loc[i] = lobj_0.Location[i];
        }
        if (!lobj_0.b_Kdtree->IsIndexed) {
          b_this = lobj_0.b_Kdtree;
          b_lobj_0.set_size(lobj_0.Location.size(0), 3);
          jA = lobj_0.Location.size(0) * lobj_0.Location.size(1) - 1;
          for (i = 0; i <= jA; i++) {
            b_lobj_0[i] = lobj_0.Location[i];
          }
          b_this->buildIndex(b_lobj_0);
          b_this->IsIndexed = true;
        }
        lobj_0.b_Kdtree->knnSearch(
            c_loc, std::fmin(6.0, static_cast<double>(exponent)), indices,
            tempNormals, valid);
        tempNormals.set_size(c_loc.size(0), 3);
        PCANormalImpl_double(&c_loc[0], &indices[0], &(valid.data())[0],
                             static_cast<unsigned int>(c_loc.size(0)),
                             static_cast<unsigned int>(indices.size(0)),
                             &tempNormals[0]);
      }
    }
  }
  pct.init(loc, pc->Color, tempNormals, pc->Intensity, &lobj_1);
  lobj_0.matlabCodegenDestructor();
  pct.get_XLimits(tempNormals);
  pct.get_XLimits(r);
  box[0] = tempNormals[1] - r[0];
  pct.get_YLimits(tempNormals);
  pct.get_YLimits(r);
  box[1] = tempNormals[1] - r[0];
  pct.get_ZLimits(tempNormals);
  pct.get_ZLimits(r);
  box[2] = tempNormals[1] - r[0];
}

//
// File trailer for PointCloudSegmentation.cpp
//
// [EOF]
//
