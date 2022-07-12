//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pctransform.cpp
//
// Code generation for function 'pctransform'
//

// Include files
#include "pctransform.h"
#include "Kdtree.h"
#include "affine3d.h"
#include "minOrMax.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "proc_mapping_rtwutil.h"
#include "rigid3d.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "svd1.h"
#include "PCANormalCore_api.hpp"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <math.h>
#include <string.h>

// Function Definitions
namespace coder {
pointCloud *pctransform(const pointCloud *ptCloudIn, const rigid3d *tform,
                        vision::internal::codegen::Kdtree *iobj_0,
                        pointCloud *iobj_1)
{
  d_pointCloud lobj_0;
  vision::internal::codegen::b_Kdtree *b_this;
  array<double, 2U> b_lobj_0;
  array<double, 2U> b_loc;
  array<double, 2U> loc;
  array<double, 2U> tempNormals;
  array<double, 1U> c_loc;
  array<unsigned int, 2U> indices;
  array<unsigned int, 1U> valid;
  array<unsigned char, 2U> b_ptCloudIn;
  double x[16];
  int boffset;
  int coffset;
  int exponent;
  int i;
  int jA;
  bool b;
  bool isodd;
  lobj_0.matlabCodegenIsDeleted = true;
  loc.set_size(ptCloudIn->Location.size(0), 3);
  jA = ptCloudIn->Location.size(0) * 3;
  for (i = 0; i < jA; i++) {
    loc[i] = ptCloudIn->Location[i];
  }
  jA = loc.size(0);
  b_loc.set_size(loc.size(0), 3);
  for (int j{0}; j < 3; j++) {
    coffset = j * jA;
    boffset = j * 3;
    for (int b_i{0}; b_i < jA; b_i++) {
      b_loc[coffset + b_i] =
          (loc[b_i] *
               tform->AffineTform
                   .T[boffset % 3 + (div_nzp_s32_floor(boffset, 3) << 2)] +
           loc[loc.size(0) + b_i] *
               tform->AffineTform.T[(boffset + 1) % 3 +
                                    (div_nzp_s32_floor(boffset + 1, 3) << 2)]) +
          loc[(loc.size(0) << 1) + b_i] *
              tform->AffineTform.T[(boffset + 2) % 3 +
                                   (div_nzp_s32_floor(boffset + 2, 3) << 2)];
    }
  }
  jA = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (i = 0; i <= jA; i++) {
    c_loc[i] = b_loc[i] + tform->AffineTform.T[3];
  }
  jA = c_loc.size(0);
  for (i = 0; i < jA; i++) {
    b_loc[i] = c_loc[i];
  }
  jA = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (i = 0; i <= jA; i++) {
    c_loc[i] = b_loc[i + b_loc.size(0)] + tform->AffineTform.T[7];
  }
  jA = c_loc.size(0);
  for (i = 0; i < jA; i++) {
    b_loc[i + b_loc.size(0)] = c_loc[i];
  }
  jA = b_loc.size(0) - 1;
  c_loc.set_size(b_loc.size(0));
  for (i = 0; i <= jA; i++) {
    c_loc[i] = b_loc[i + b_loc.size(0) * 2] + tform->AffineTform.T[11];
  }
  jA = c_loc.size(0);
  for (i = 0; i < jA; i++) {
    b_loc[i + b_loc.size(0) * 2] = c_loc[i];
  }
  tempNormals.set_size(0, 0);
  isodd = (ptCloudIn->Normal.size(0) == 0);
  b = (ptCloudIn->Normal.size(1) == 0);
  if ((!isodd) && (!b)) {
    double b_tform[9];
    double singularValues[3];
    double absx;
    double s;
    bool guard1{false};
    for (i = 0; i < 3; i++) {
      jA = i << 2;
      b_tform[3 * i] = tform->AffineTform.T[jA];
      b_tform[3 * i + 1] = tform->AffineTform.T[jA + 1];
      b_tform[3 * i + 2] = tform->AffineTform.T[jA + 2];
    }
    svd(b_tform, singularValues);
    s = internal::maximum(singularValues);
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
    if (s - internal::minimum(singularValues) < 100.0 * absx) {
      int jp1j;
      signed char ipiv[4];
      std::copy(&tform->AffineTform.T[0], &tform->AffineTform.T[16], &x[0]);
      ipiv[0] = 1;
      ipiv[1] = 2;
      ipiv[2] = 3;
      for (int j{0}; j < 3; j++) {
        coffset = 2 - j;
        boffset = j * 5;
        jp1j = boffset + 2;
        jA = 4 - j;
        exponent = 0;
        absx = std::abs(x[boffset]);
        for (int k{2}; k <= jA; k++) {
          s = std::abs(x[(boffset + k) - 1]);
          if (s > absx) {
            exponent = k - 1;
            absx = s;
          }
        }
        if (x[boffset + exponent] != 0.0) {
          if (exponent != 0) {
            jA = j + exponent;
            ipiv[j] = static_cast<signed char>(jA + 1);
            absx = x[j];
            x[j] = x[jA];
            x[jA] = absx;
            absx = x[j + 4];
            x[j + 4] = x[jA + 4];
            x[jA + 4] = absx;
            absx = x[j + 8];
            x[j + 8] = x[jA + 8];
            x[jA + 8] = absx;
            absx = x[j + 12];
            x[j + 12] = x[jA + 12];
            x[jA + 12] = absx;
          }
          i = (boffset - j) + 4;
          for (int b_i{jp1j}; b_i <= i; b_i++) {
            x[b_i - 1] /= x[boffset];
          }
        }
        jA = boffset;
        for (jp1j = 0; jp1j <= coffset; jp1j++) {
          absx = x[(boffset + (jp1j << 2)) + 4];
          if (absx != 0.0) {
            i = jA + 6;
            exponent = (jA - j) + 8;
            for (int b_i{i}; b_i <= exponent; b_i++) {
              x[b_i - 1] += x[((boffset + b_i) - jA) - 5] * -absx;
            }
          }
          jA += 4;
        }
      }
      isodd = (ipiv[0] > 1);
      if (ipiv[1] > 2) {
        isodd = !isodd;
      }
      absx = x[0] * x[5] * x[10] * x[15];
      if (ipiv[2] > 3) {
        isodd = !isodd;
      }
      if (isodd) {
        absx = -absx;
      }
      if (std::abs(absx - 1.0) < 2.2204460492503131E-14) {
        tempNormals.set_size(ptCloudIn->Normal.size(0),
                             ptCloudIn->Normal.size(1));
        jA = ptCloudIn->Normal.size(0) * ptCloudIn->Normal.size(1);
        for (i = 0; i < jA; i++) {
          tempNormals[i] = ptCloudIn->Normal[i];
        }
        exponent = tempNormals.size(0);
        jA = tempNormals.size(1);
        loc.set_size(tempNormals.size(0), 3);
        for (int j{0}; j < 3; j++) {
          coffset = j * exponent;
          boffset = j * 3;
          for (int b_i{0}; b_i < exponent; b_i++) {
            loc[coffset + b_i] = 0.0;
          }
          for (int k{0}; k < jA; k++) {
            jp1j = k * tempNormals.size(0);
            i = boffset + k;
            absx = tform->AffineTform.T[i % 3 + (div_nzp_s32_floor(i, 3) << 2)];
            for (int b_i{0}; b_i < exponent; b_i++) {
              i = coffset + b_i;
              loc[i] = loc[i] + tempNormals[jp1j + b_i] * absx;
            }
          }
        }
        tempNormals.set_size(loc.size(0), 3);
        jA = loc.size(0) * 3;
        for (i = 0; i < jA; i++) {
          tempNormals[i] = loc[i];
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      lobj_0.Location.set_size(b_loc.size(0), 3);
      jA = b_loc.size(0) * 3;
      for (i = 0; i < jA; i++) {
        lobj_0.Location[i] = b_loc[i];
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
        loc.set_size(lobj_0.Location.size(0), 3);
        jA = lobj_0.Location.size(0) * 3;
        for (i = 0; i < jA; i++) {
          loc[i] = lobj_0.Location[i];
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
            loc, std::fmin(6.0, static_cast<double>(exponent)), indices,
            tempNormals, valid);
        tempNormals.set_size(loc.size(0), 3);
        PCANormalImpl_double(&loc[0], &indices[0], &(valid.data())[0],
                             static_cast<unsigned int>(loc.size(0)),
                             static_cast<unsigned int>(indices.size(0)),
                             &tempNormals[0]);
      }
    }
  }
  b_ptCloudIn.set_size(ptCloudIn->Color.size(0), ptCloudIn->Color.size(1));
  jA = ptCloudIn->Color.size(0) * ptCloudIn->Color.size(1) - 1;
  for (i = 0; i <= jA; i++) {
    b_ptCloudIn[i] = ptCloudIn->Color[i];
  }
  c_loc.set_size(ptCloudIn->Intensity.size(0));
  jA = ptCloudIn->Intensity.size(0) - 1;
  for (i = 0; i <= jA; i++) {
    c_loc[i] = ptCloudIn->Intensity[i];
  }
  return iobj_1->init(b_loc, b_ptCloudIn, tempNormals, c_loc, iobj_0);
}

b_pointCloud *pctransform(const b_pointCloud *ptCloudIn, const rigid3d *tform,
                          vision::internal::codegen::Kdtree *iobj_0,
                          b_pointCloud *iobj_1)
{
  static e_pointCloud lobj_0;
  static float loc[22317];
  b_pointCloud *ptCloudOut;
  array<float, 2U> nv;
  float T[16];
  float x[16];
  int exponent;
  int i;
  int idx;
  int jp1j;
  for (i = 0; i < 16; i++) {
    T[i] = static_cast<float>(tform->AffineTform.T[i]);
  }
  lobj_0.matlabCodegenIsDeleted = true;
  for (i = 0; i < 7439; i++) {
    for (jp1j = 0; jp1j < 3; jp1j++) {
      idx = jp1j << 2;
      loc[i + 7439 * jp1j] = (ptCloudIn->Location[i] * T[idx] +
                              ptCloudIn->Location[i + 7439] * T[idx + 1]) +
                             ptCloudIn->Location[i + 14878] * T[idx + 2];
    }
    loc[i] += T[3];
    loc[i + 7439] += T[7];
    loc[i + 14878] += T[11];
  }
  nv.set_size(0, 0);
  if ((ptCloudIn->Normal.size(0) != 0) && (ptCloudIn->Normal.size(1) != 0)) {
    float s[3];
    float absx;
    float b_s;
    float ex;
    int k;
    bool exitg1;
    bool guard1{false};
    bool isodd;
    isodd = true;
    for (k = 0; k < 9; k++) {
      if (isodd) {
        absx = T[k % 3 + (div_nzp_s32_floor(k, 3) << 2)];
        if (std::isinf(absx) || std::isnan(absx)) {
          isodd = false;
        }
      } else {
        isodd = false;
      }
    }
    if (isodd) {
      float b_T[9];
      for (i = 0; i < 3; i++) {
        idx = i << 2;
        b_T[3 * i] = T[idx];
        b_T[3 * i + 1] = T[idx + 1];
        b_T[3 * i + 2] = T[idx + 2];
      }
      internal::b_svd(b_T, s);
    } else {
      s[0] = rtNaNF;
      s[1] = rtNaNF;
      s[2] = rtNaNF;
    }
    isodd = std::isnan(s[0]);
    if (!isodd) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 3)) {
        if (!std::isnan(s[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      b_s = s[0];
    } else {
      b_s = s[idx - 1];
      i = idx + 1;
      for (k = i; k < 4; k++) {
        absx = s[k - 1];
        if (b_s < absx) {
          b_s = absx;
        }
      }
    }
    if (!isodd) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 3)) {
        if (!std::isnan(s[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = s[0];
    } else {
      ex = s[idx - 1];
      i = idx + 1;
      for (k = i; k < 4; k++) {
        absx = s[k - 1];
        if (ex > absx) {
          ex = absx;
        }
      }
    }
    absx = std::abs(b_s);
    if ((!std::isinf(absx)) && (!std::isnan(absx))) {
      if (absx <= 1.17549435E-38F) {
        absx = 1.4013E-45F;
      } else {
        std::frexp(absx, &exponent);
        absx = std::ldexp(1.0F, exponent - 24);
      }
    } else {
      absx = rtNaNF;
    }
    guard1 = false;
    if (b_s - ex < 100.0F * absx) {
      int aoffset;
      int boffset;
      signed char ipiv[4];
      std::copy(&T[0], &T[16], &x[0]);
      ipiv[0] = 1;
      ipiv[1] = 2;
      ipiv[2] = 3;
      for (int j{0}; j < 3; j++) {
        int b_tmp;
        aoffset = 2 - j;
        b_tmp = j * 5;
        jp1j = b_tmp + 2;
        idx = 4 - j;
        exponent = 0;
        absx = std::abs(x[b_tmp]);
        for (k = 2; k <= idx; k++) {
          b_s = std::abs(x[(b_tmp + k) - 1]);
          if (b_s > absx) {
            exponent = k - 1;
            absx = b_s;
          }
        }
        if (x[b_tmp + exponent] != 0.0F) {
          if (exponent != 0) {
            idx = j + exponent;
            ipiv[j] = static_cast<signed char>(idx + 1);
            absx = x[j];
            x[j] = x[idx];
            x[idx] = absx;
            absx = x[j + 4];
            x[j + 4] = x[idx + 4];
            x[idx + 4] = absx;
            absx = x[j + 8];
            x[j + 8] = x[idx + 8];
            x[idx + 8] = absx;
            absx = x[j + 12];
            x[j + 12] = x[idx + 12];
            x[idx + 12] = absx;
          }
          i = (b_tmp - j) + 4;
          for (int b_i{jp1j}; b_i <= i; b_i++) {
            x[b_i - 1] /= x[b_tmp];
          }
        }
        idx = b_tmp;
        for (exponent = 0; exponent <= aoffset; exponent++) {
          absx = x[(b_tmp + (exponent << 2)) + 4];
          if (absx != 0.0F) {
            i = idx + 6;
            jp1j = (idx - j) + 8;
            for (boffset = i; boffset <= jp1j; boffset++) {
              x[boffset - 1] += x[((b_tmp + boffset) - idx) - 5] * -absx;
            }
          }
          idx += 4;
        }
      }
      isodd = (ipiv[0] > 1);
      if (ipiv[1] > 2) {
        isodd = !isodd;
      }
      absx = x[0] * x[5] * x[10] * x[15];
      if (ipiv[2] > 3) {
        isodd = !isodd;
      }
      if (isodd) {
        absx = -absx;
      }
      if (std::abs(absx - 1.0F) < 1.1920929E-5F) {
        idx = ptCloudIn->Normal.size(0);
        exponent = ptCloudIn->Normal.size(1);
        nv.set_size(ptCloudIn->Normal.size(0), 3);
        for (int j{0}; j < 3; j++) {
          jp1j = j * idx;
          boffset = j * 3;
          for (int b_i{0}; b_i < idx; b_i++) {
            nv[jp1j + b_i] = 0.0F;
          }
          for (k = 0; k < exponent; k++) {
            aoffset = k * ptCloudIn->Normal.size(0);
            i = boffset + k;
            absx = T[i % 3 + (div_nzp_s32_floor(i, 3) << 2)];
            for (int b_i{0}; b_i < idx; b_i++) {
              i = jp1j + b_i;
              nv[i] = nv[i] + ptCloudIn->Normal[aoffset + b_i] * absx;
            }
          }
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      std::copy(&loc[0], &loc[22317], &lobj_0.Location[0]);
      lobj_0.Color.set_size(0, 0);
      lobj_0.Normal.set_size(0, 0);
      lobj_0.Intensity.set_size(0, 0);
      lobj_0._pobj0.InputData.set_size(0, 0);
      lobj_0._pobj0.NxNoNaN = 0.0;
      lobj_0._pobj0.CutDim.set_size(0, 0);
      lobj_0._pobj0.CutVal.set_size(0, 0);
      lobj_0._pobj0.LowerBounds.set_size(0, 0);
      lobj_0._pobj0.UpperBounds.set_size(0, 0);
      lobj_0._pobj0.LeftChild.set_size(0, 0);
      lobj_0._pobj0.RightChild.set_size(0, 0);
      lobj_0._pobj0.LeafNode.set_size(0, 0);
      lobj_0._pobj0.IdxAll.set_size(0);
      lobj_0._pobj0.IdxDim.set_size(0);
      lobj_0._pobj0.IsIndexed = false;
      lobj_0.b_Kdtree = &lobj_0._pobj0;
      lobj_0.matlabCodegenIsDeleted = false;
      lobj_0.surfaceNormalImpl(nv);
    }
  }
  ptCloudOut = iobj_1;
  for (i = 0; i < 22317; i++) {
    iobj_1->Location[i] = loc[i];
  }
  iobj_1->Color.set_size(ptCloudIn->Color.size(0), ptCloudIn->Color.size(1));
  idx = ptCloudIn->Color.size(0) * ptCloudIn->Color.size(1);
  for (i = 0; i < idx; i++) {
    iobj_1->Color[i] = ptCloudIn->Color[i];
  }
  iobj_1->Normal.set_size(nv.size(0), nv.size(1));
  idx = nv.size(0) * nv.size(1);
  for (i = 0; i < idx; i++) {
    iobj_1->Normal[i] = nv[i];
  }
  iobj_1->Intensity.set_size(ptCloudIn->Intensity.size(0),
                             ptCloudIn->Intensity.size(1));
  idx = ptCloudIn->Intensity.size(0) * ptCloudIn->Intensity.size(1);
  for (i = 0; i < idx; i++) {
    iobj_1->Intensity[i] = ptCloudIn->Intensity[i];
  }
  pointclouds::internal::codegen::pc::pointCloudArray r;
  iobj_1->RangeData.set_size(0, 0);
  iobj_1->PointCloudArrayData.set_size(1, 1);
  iobj_1->PointCloudArrayData[0] = r;
  iobj_1->Kdtree = iobj_0;
  iobj_1->matlabCodegenIsDeleted = false;
  return ptCloudOut;
}

} // namespace coder

// End of code generation (pctransform.cpp)
