//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcregistericp.cpp
//
// Code generation for function 'pcregistericp'
//

// Include files
#include "pcregistericp.h"
#include "Kdtree.h"
#include "affine3d.h"
#include "any1.h"
#include "computeRigidTransform.h"
#include "find.h"
#include "minOrMax.h"
#include "pointCloud.h"
#include "ref.h"
#include "rigid3d.h"
#include "rigid3dImpl.h"
#include "rotationToQuaternion.h"
#include "rt_nonfinite.h"
#include "sortedInsertion.h"
#include "svd.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>
#include <math.h>
#include <string.h>

// Function Declarations
namespace coder {
static void getChangesInTransformation(const e_captured_var *i,
                                       const c_captured_var *qs,
                                       const b_captured_var *Ts, double *dR,
                                       double *dT, double *rdiff,
                                       double *tdiff);

static void parseInputs(double varargin_2, double *inlierRatio,
                        double tolerance[2], rigid3d *initTformParsed);

static void preparePointClouds(const b_pointCloud *moving,
                               const pointCloud *fixed, c_pointCloud *iobj_0,
                               f_pointCloud *iobj_1, f_pointCloud **ptCloudA,
                               c_pointCloud **ptCloudB);

} // namespace coder

// Function Definitions
namespace coder {
static void getChangesInTransformation(const e_captured_var *i,
                                       const c_captured_var *qs,
                                       const b_captured_var *Ts, double *dR,
                                       double *dT, double *rdiff, double *tdiff)
{
  double count;
  double d;
  int b_i;
  *dR = 0.0;
  *dT = 0.0;
  *rdiff = 0.0;
  *tdiff = 0.0;
  count = 0.0;
  d = std::fmax(i->contents - 2.0, 1.0);
  b_i = static_cast<int>(i->contents + (1.0 - d));
  for (int k{0}; k < b_i; k++) {
    double absxk;
    double b_c_tmp;
    double b_k;
    double b_scale;
    double b_y;
    double d_c_tmp;
    double e_c_tmp;
    double f_c_tmp;
    double g_c_tmp;
    double h_c_tmp;
    double i_c_tmp;
    double j_c_tmp;
    double scale;
    double t;
    double y;
    int c_c_tmp;
    int c_tmp;
    b_k = d + static_cast<double>(k);
    scale = 3.3121686421112381E-170;
    b_scale = 3.3121686421112381E-170;
    c_tmp = 7 * (static_cast<int>(b_k) - 1);
    b_c_tmp = qs->contents[c_tmp];
    c_c_tmp = 7 * static_cast<int>(b_k);
    d_c_tmp = qs->contents[c_c_tmp];
    absxk = std::abs(b_c_tmp);
    if (absxk > 3.3121686421112381E-170) {
      y = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      y = t * t;
    }
    absxk = std::abs(d_c_tmp);
    if (absxk > 3.3121686421112381E-170) {
      b_y = 1.0;
      b_scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      b_y = t * t;
    }
    e_c_tmp = qs->contents[c_tmp + 1];
    f_c_tmp = qs->contents[c_c_tmp + 1];
    absxk = std::abs(e_c_tmp);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
    absxk = std::abs(f_c_tmp);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      b_y = b_y * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      b_y += t * t;
    }
    g_c_tmp = qs->contents[c_tmp + 2];
    h_c_tmp = qs->contents[c_c_tmp + 2];
    absxk = std::abs(g_c_tmp);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
    absxk = std::abs(h_c_tmp);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      b_y = b_y * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      b_y += t * t;
    }
    i_c_tmp = qs->contents[c_tmp + 3];
    j_c_tmp = qs->contents[c_c_tmp + 3];
    absxk = std::abs(i_c_tmp);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
    absxk = std::abs(j_c_tmp);
    if (absxk > b_scale) {
      t = b_scale / absxk;
      b_y = b_y * t * t + 1.0;
      b_scale = absxk;
    } else {
      t = absxk / b_scale;
      b_y += t * t;
    }
    y = scale * std::sqrt(y);
    b_y = b_scale * std::sqrt(b_y);
    *rdiff = std::acos(std::fmin(
        1.0, std::fmax(-1.0, (((b_c_tmp * d_c_tmp + e_c_tmp * f_c_tmp) +
                               g_c_tmp * h_c_tmp) +
                              i_c_tmp * j_c_tmp) /
                                 (y * b_y))));
    c_tmp = 3 * (static_cast<int>(b_k) - 1);
    c_c_tmp = 3 * static_cast<int>(b_k);
    scale = Ts->contents[c_tmp] - Ts->contents[c_c_tmp];
    b_scale = scale * scale;
    scale = Ts->contents[c_tmp + 1] - Ts->contents[c_c_tmp + 1];
    b_c_tmp = scale * scale;
    scale = Ts->contents[c_tmp + 2] - Ts->contents[c_c_tmp + 2];
    *tdiff = std::sqrt((b_scale + b_c_tmp) + scale * scale);
    *dR += *rdiff;
    *dT += *tdiff;
    count++;
  }
  *dT /= count;
  *dR /= count;
}

static void parseInputs(double varargin_2, double *inlierRatio,
                        double tolerance[2], rigid3d *initTformParsed)
{
  static const signed char T[16]{1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1};
  double b_initTformParsed[9];
  double absx;
  int exponent;
  for (int i{0}; i < 16; i++) {
    initTformParsed->AffineTform.T[i] = T[i];
  }
  images::internal::rigid3dImpl r;
  initTformParsed->Data.set_size(1, 1);
  initTformParsed->Data[0] = r;
  for (int i{0}; i < 3; i++) {
    int initTformParsed_tmp;
    initTformParsed_tmp = i << 2;
    b_initTformParsed[3 * i] =
        initTformParsed->AffineTform.T[initTformParsed_tmp];
    b_initTformParsed[3 * i + 1] =
        initTformParsed->AffineTform.T[initTformParsed_tmp + 1];
    b_initTformParsed[3 * i + 2] =
        initTformParsed->AffineTform.T[initTformParsed_tmp + 2];
  }
  double dv[3];
  svd(b_initTformParsed, dv);
  absx = std::abs(internal::maximum(dv));
  if ((!std::isinf(absx)) && (!std::isnan(absx)) &&
      (!(absx <= 2.2250738585072014E-308))) {
    frexp(absx, &exponent);
  }
  tolerance[0] = 0.01;
  tolerance[1] = 0.0087266462599716477;
  *inlierRatio = varargin_2;
}

static void preparePointClouds(const b_pointCloud *moving,
                               const pointCloud *fixed, c_pointCloud *iobj_0,
                               f_pointCloud *iobj_1, f_pointCloud **ptCloudA,
                               c_pointCloud **ptCloudB)
{
  c_pointCloud *b_ptCloudB;
  array<double, 1U> validPtCloudIndices;
  array<float, 2U> intensity;
  array<float, 2U> tempI;
  array<float, 2U> tempNV;
  array<short, 1U> r;
  array<short, 1U> r1;
  array<short, 1U> r2;
  array<short, 1U> r3;
  array<unsigned char, 2U> C_;
  array<unsigned char, 2U> color;
  float x[22317];
  int nz[7439];
  int i;
  int xoffset;
  bool tf[22317];
  bool indices[7439];
  for (int k{0}; k < 22317; k++) {
    x[k] = moving->Location[k];
  }
  for (int k{0}; k < 22317; k++) {
    float f;
    f = x[k];
    tf[k] = ((!std::isinf(f)) && (!std::isnan(f)));
  }
  for (i = 0; i < 7439; i++) {
    nz[i] = tf[i];
  }
  for (int k{0}; k < 2; k++) {
    xoffset = (k + 1) * 7439;
    for (i = 0; i < 7439; i++) {
      nz[i] += tf[xoffset + i];
    }
  }
  for (i = 0; i < 7439; i++) {
    indices[i] = (nz[i] == 3);
  }
  for (int k{0}; k < 22317; k++) {
    x[k] = moving->Location[k];
  }
  color.set_size(moving->Color.size(0), moving->Color.size(1));
  xoffset = moving->Color.size(0) * moving->Color.size(1);
  for (int k{0}; k < xoffset; k++) {
    color[k] = moving->Color[k];
  }
  tempI.set_size(moving->Normal.size(0), moving->Normal.size(1));
  xoffset = moving->Normal.size(0) * moving->Normal.size(1);
  for (int k{0}; k < xoffset; k++) {
    tempI[k] = moving->Normal[k];
  }
  intensity.set_size(moving->Intensity.size(0), moving->Intensity.size(1));
  xoffset = moving->Intensity.size(0) * moving->Intensity.size(1);
  for (int k{0}; k < xoffset; k++) {
    intensity[k] = moving->Intensity[k];
  }
  xoffset = 0;
  for (i = 0; i < 7439; i++) {
    if (indices[i]) {
      xoffset++;
    }
  }
  r.set_size(xoffset);
  xoffset = 0;
  for (i = 0; i < 7439; i++) {
    if (indices[i]) {
      r[xoffset] = static_cast<short>(i + 1);
      xoffset++;
    }
  }
  if ((color.size(0) != 0) && (color.size(1) != 0)) {
    xoffset = 0;
    for (i = 0; i < 7439; i++) {
      if (indices[i]) {
        xoffset++;
      }
    }
    r1.set_size(xoffset);
    xoffset = 0;
    for (i = 0; i < 7439; i++) {
      if (indices[i]) {
        r1[xoffset] = static_cast<short>(i + 1);
        xoffset++;
      }
    }
    xoffset = color.size(1);
    C_.set_size(r1.size(0), color.size(1));
    i = r1.size(0);
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < i; b_i++) {
        C_[b_i + C_.size(0) * k] = color[(r1[b_i] + color.size(0) * k) - 1];
      }
    }
  } else {
    C_.set_size(0, 0);
  }
  if ((tempI.size(0) != 0) && (tempI.size(1) != 0)) {
    xoffset = 0;
    for (i = 0; i < 7439; i++) {
      if (indices[i]) {
        xoffset++;
      }
    }
    r2.set_size(xoffset);
    xoffset = 0;
    for (i = 0; i < 7439; i++) {
      if (indices[i]) {
        r2[xoffset] = static_cast<short>(i + 1);
        xoffset++;
      }
    }
    xoffset = tempI.size(1);
    tempNV.set_size(r2.size(0), tempI.size(1));
    i = r2.size(0);
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < i; b_i++) {
        tempNV[b_i + tempNV.size(0) * k] =
            tempI[(r2[b_i] + tempI.size(0) * k) - 1];
      }
    }
  } else {
    tempNV.set_size(0, 0);
  }
  if ((intensity.size(0) != 0) && (intensity.size(1) != 0)) {
    xoffset = 0;
    for (i = 0; i < 7439; i++) {
      if (indices[i]) {
        xoffset++;
      }
    }
    r3.set_size(xoffset);
    xoffset = 0;
    for (i = 0; i < 7439; i++) {
      if (indices[i]) {
        r3[xoffset] = static_cast<short>(i + 1);
        xoffset++;
      }
    }
    xoffset = intensity.size(1);
    tempI.set_size(r3.size(0), intensity.size(1));
    i = r3.size(0);
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < i; b_i++) {
        tempI[b_i + tempI.size(0) * k] =
            intensity[(r3[b_i] + intensity.size(0) * k) - 1];
      }
    }
  } else {
    tempI.set_size(0, 0);
  }
  *ptCloudA = iobj_1;
  iobj_1->Location.set_size(r.size(0), 3);
  xoffset = r.size(0);
  for (int k{0}; k < 3; k++) {
    for (int b_i{0}; b_i < xoffset; b_i++) {
      iobj_1->Location[b_i + iobj_1->Location.size(0) * k] =
          x[(r[b_i] + 7439 * k) - 1];
    }
  }
  iobj_1->Color.set_size(C_.size(0), C_.size(1));
  xoffset = C_.size(0) * C_.size(1);
  for (int k{0}; k < xoffset; k++) {
    iobj_1->Color[k] = C_[k];
  }
  iobj_1->Normal.set_size(tempNV.size(0), tempNV.size(1));
  xoffset = tempNV.size(0) * tempNV.size(1);
  for (int k{0}; k < xoffset; k++) {
    iobj_1->Normal[k] = tempNV[k];
  }
  iobj_1->Intensity.set_size(tempI.size(0), tempI.size(1));
  xoffset = tempI.size(0) * tempI.size(1);
  for (int k{0}; k < xoffset; k++) {
    iobj_1->Intensity[k] = tempI[k];
  }
  iobj_1->b_Kdtree = &iobj_1->_pobj0;
  iobj_1->matlabCodegenIsDeleted = false;
  fixed->removeInvalidPoints(iobj_0, &b_ptCloudB, validPtCloudIndices);
  *ptCloudB = b_ptCloudB;
}

void pcregistericp(b_pointCloud *moving, pointCloud *fixed, double varargin_2,
                   rigid3d *tform)
{
  static const signed char iv[147]{
      1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
  b_captured_var Ts;
  c_captured_var qs;
  c_pointCloud ptCloudB;
  c_pointCloud *r2;
  captured_var Rs;
  d_captured_var locA;
  e_captured_var c_i;
  f_captured_var ptCloudA;
  f_pointCloud r;
  f_pointCloud *r1;
  images::internal::rigid3dImpl r3;
  rigid3d initialTransform;
  vision::internal::codegen::b_Kdtree *b_this;
  array<double, 2U> b_ptCloudB;
  array<double, 2U> inlierDist;
  array<double, 1U> b;
  array<double, 1U> xv;
  array<float, 2U> b_locA;
  array<int, 2U> ii;
  array<unsigned int, 2U> indices;
  array<unsigned int, 2U> inlierIndicesB;
  array<int, 2U> r4;
  array<int, 1U> itmp;
  array<int, 1U> iwork;
  array<unsigned int, 1U> valid;
  array<int, 1U> yv;
  array<bool, 1U> keepInlierA;
  double _2[16];
  double U[9];
  double V[9];
  double a__2[9];
  double b_Rs[9];
  double dv[4];
  double tolerance[2];
  double dT;
  double inlierRatio;
  double rdiff;
  double tdiff;
  double upperBound;
  float s;
  int b_i;
  int i;
  int i2;
  int len;
  int loop_ub;
  int numPointsA;
  int outsize_idx_0;
  int p;
  int qEnd;
  int stopIteration;
  int unnamed_idx_0;
  bool exitg1;
  bool useAllMatches;
  r.matlabCodegenIsDeleted = true;
  ptCloudB.matlabCodegenIsDeleted = true;
  ptCloudA.matlabCodegenIsDeleted = false;
  parseInputs(varargin_2, &inlierRatio, tolerance, &initialTransform);
  useAllMatches = (inlierRatio == 1.0);
  preparePointClouds(moving, fixed, &ptCloudB, &r, &r1, &r2);
  ptCloudA.contents = &r;
  std::memset(&Rs.contents[0], 0, 189U * sizeof(double));
  std::memset(&Ts.contents[0], 0, 63U * sizeof(double));
  for (i = 0; i < 147; i++) {
    qs.contents[i] = iv[i];
  }
  for (i = 0; i < 3; i++) {
    Rs.contents[3 * i] = initialTransform.AffineTform.T[i];
    Rs.contents[3 * i + 1] = initialTransform.AffineTform.T[i + 4];
    Rs.contents[3 * i + 2] = initialTransform.AffineTform.T[i + 8];
    Ts.contents[i] = initialTransform.AffineTform.T[(i << 2) + 3];
  }
  vision::internal::quaternion::rotationToQuaternion(
      *(double(*)[9]) & Rs.contents[0], dv);
  qs.contents[0] = dv[0];
  qs.contents[1] = dv[1];
  qs.contents[2] = dv[2];
  qs.contents[3] = dv[3];
  qs.contents[4] = Ts.contents[0];
  qs.contents[5] = Ts.contents[1];
  qs.contents[6] = Ts.contents[2];
  locA.contents.set_size(r.Location.size(0), 3);
  i2 = r.Location.size(0) * 3;
  for (i = 0; i < i2; i++) {
    locA.contents[i] = r.Location[i];
  }
  if ((dv[0] != 0.0) || c_any(*(double(*)[6]) & qs.contents[1])) {
    b_locA.set_size(r.Location.size(0), 3);
    i2 = r.Location.size(0);
    for (i = 0; i < i2; i++) {
      for (p = 0; p < 3; p++) {
        s = r.Location[i] * static_cast<float>(Rs.contents[p]);
        s += r.Location[i + r.Location.size(0)] *
             static_cast<float>(Rs.contents[p + 3]);
        s += r.Location[i + r.Location.size(0) * 2] *
             static_cast<float>(Rs.contents[p + 6]);
        b_locA[i + b_locA.size(0) * p] = s;
      }
    }
    locA.contents.set_size(b_locA.size(0), 3);
    i2 = b_locA.size(0);
    for (i = 0; i < 3; i++) {
      for (p = 0; p < i2; p++) {
        locA.contents[p + locA.contents.size(0) * i] =
            b_locA[p + b_locA.size(0) * i] + static_cast<float>(Ts.contents[i]);
      }
    }
  }
  stopIteration = 20;
  numPointsA = r.Location.size(0);
  upperBound =
      std::fmax(1.0, std::round(inlierRatio * static_cast<double>(numPointsA)));
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 20)) {
    float R[9];
    float Rnew[9];
    float T[3];
    float Tnew[3];
    int d_i;
    c_i.contents = static_cast<double>(b_i) + 1.0;
    if (useAllMatches) {
      xv.set_size(static_cast<int>(upperBound - 1.0) + 1);
      i2 = static_cast<int>(upperBound - 1.0);
      for (i = 0; i <= i2; i++) {
        xv[i] = static_cast<double>(i) + 1.0;
      }
      len = ptCloudB.Location.size(0);
      if (!ptCloudB.b_Kdtree->IsIndexed) {
        b_this = ptCloudB.b_Kdtree;
        b_ptCloudB.set_size(ptCloudB.Location.size(0), 3);
        i2 = ptCloudB.Location.size(0) * ptCloudB.Location.size(1) - 1;
        for (i = 0; i <= i2; i++) {
          b_ptCloudB[i] = ptCloudB.Location[i];
        }
        b_this->buildIndex(b_ptCloudB);
        b_this->IsIndexed = true;
      }
      b_ptCloudB.set_size(locA.contents.size(0), 3);
      i2 = locA.contents.size(0) * 3;
      for (i = 0; i < i2; i++) {
        b_ptCloudB[i] = locA.contents[i];
      }
      ptCloudB.b_Kdtree->knnSearch(b_ptCloudB,
                                   std::fmin(1.0, static_cast<double>(len)),
                                   inlierIndicesB, inlierDist, valid);
    } else {
      int k;
      int nx;
      int stride;
      len = ptCloudB.Location.size(0);
      if (!ptCloudB.b_Kdtree->IsIndexed) {
        b_this = ptCloudB.b_Kdtree;
        b_ptCloudB.set_size(ptCloudB.Location.size(0), 3);
        i2 = ptCloudB.Location.size(0) * ptCloudB.Location.size(1) - 1;
        for (i = 0; i <= i2; i++) {
          b_ptCloudB[i] = ptCloudB.Location[i];
        }
        b_this->buildIndex(b_ptCloudB);
        b_this->IsIndexed = true;
      }
      b_ptCloudB.set_size(locA.contents.size(0), 3);
      i2 = locA.contents.size(0) * 3;
      for (i = 0; i < i2; i++) {
        b_ptCloudB[i] = locA.contents[i];
      }
      ptCloudB.b_Kdtree->knnSearch(b_ptCloudB,
                                   std::fmin(1.0, static_cast<double>(len)),
                                   indices, inlierDist, valid);
      keepInlierA.set_size(numPointsA);
      for (i = 0; i < numPointsA; i++) {
        keepInlierA[i] = false;
      }
      if (upperBound <= inlierDist.size(1)) {
        k = static_cast<int>(upperBound);
      } else {
        k = inlierDist.size(1);
      }
      ii.set_size(inlierDist.size(0), k);
      i2 = inlierDist.size(0) * k;
      for (i = 0; i < i2; i++) {
        ii[i] = 0;
      }
      nx = inlierDist.size(1);
      stride = inlierDist.size(0);
      if (inlierDist.size(0) - 1 >= 0) {
        outsize_idx_0 = inlierDist.size(1);
        loop_ub = inlierDist.size(1);
        unnamed_idx_0 = k;
      }
      for (int j{0}; j < stride; j++) {
        int b_k;
        int n;
        xv.set_size(outsize_idx_0);
        for (i = 0; i < loop_ub; i++) {
          xv[i] = 0.0;
        }
        for (b_k = 0; b_k < nx; b_k++) {
          xv[b_k] = inlierDist[j + b_k * stride];
        }
        n = xv.size(0);
        yv.set_size(unnamed_idx_0);
        for (i = 0; i < unnamed_idx_0; i++) {
          yv[i] = 0;
        }
        b.set_size(unnamed_idx_0);
        for (i = 0; i < unnamed_idx_0; i++) {
          b[i] = 0.0;
        }
        if (k != 0) {
          if ((k > 64) && (k > (xv.size(0) >> 6))) {
            int b_j;
            n = xv.size(0) + 1;
            itmp.set_size(xv.size(0));
            i2 = xv.size(0);
            for (i = 0; i < i2; i++) {
              itmp[i] = 0;
            }
            if (xv.size(0) != 0) {
              iwork.set_size(xv.size(0));
              i = xv.size(0) - 1;
              for (b_k = 1; b_k <= i; b_k += 2) {
                if ((xv[b_k - 1] <= xv[b_k]) || std::isnan(xv[b_k])) {
                  itmp[b_k - 1] = b_k;
                  itmp[b_k] = b_k + 1;
                } else {
                  itmp[b_k - 1] = b_k + 1;
                  itmp[b_k] = b_k;
                }
              }
              if ((xv.size(0) & 1) != 0) {
                itmp[xv.size(0) - 1] = xv.size(0);
              }
              d_i = 2;
              while (d_i < n - 1) {
                i2 = d_i << 1;
                b_j = 1;
                for (len = d_i + 1; len < n; len = qEnd + d_i) {
                  int kEnd;
                  int q;
                  p = b_j;
                  q = len - 1;
                  qEnd = b_j + i2;
                  if (qEnd > n) {
                    qEnd = n;
                  }
                  b_k = 0;
                  kEnd = qEnd - b_j;
                  while (b_k + 1 <= kEnd) {
                    rdiff = xv[itmp[q] - 1];
                    i = itmp[p - 1];
                    if ((xv[i - 1] <= rdiff) || std::isnan(rdiff)) {
                      iwork[b_k] = i;
                      p++;
                      if (p == len) {
                        while (q + 1 < qEnd) {
                          b_k++;
                          iwork[b_k] = itmp[q];
                          q++;
                        }
                      }
                    } else {
                      iwork[b_k] = itmp[q];
                      q++;
                      if (q + 1 == qEnd) {
                        while (p < len) {
                          b_k++;
                          iwork[b_k] = itmp[p - 1];
                          p++;
                        }
                      }
                    }
                    b_k++;
                  }
                  for (b_k = 0; b_k < kEnd; b_k++) {
                    itmp[(b_j + b_k) - 1] = iwork[b_k];
                  }
                  b_j = qEnd;
                }
                d_i = i2;
              }
            }
            yv.set_size(unnamed_idx_0);
            for (b_j = 0; b_j < k; b_j++) {
              yv[b_j] = itmp[b_j];
            }
          } else {
            for (int b_j{0}; b_j < k; b_j++) {
              i2 = b_j;
              internal::sortedInsertion(xv[b_j], b_j + 1, b, &i2, k, yv);
            }
            i = k + 1;
            for (int b_j{i}; b_j <= n; b_j++) {
              i2 = k;
              internal::sortedInsertion(xv[b_j - 1], b_j, b, &i2, k, yv);
            }
          }
        }
        len = yv.size(0);
        for (b_k = 0; b_k < len; b_k++) {
          ii[j + b_k * stride] = yv[b_k];
        }
      }
      i2 = ii.size(0) * ii.size(1);
      for (i = 0; i < i2; i++) {
        keepInlierA[ii[i] - 1] = true;
      }
      b_eml_find(keepInlierA, yv);
      xv.set_size(yv.size(0));
      i2 = yv.size(0);
      for (i = 0; i < i2; i++) {
        xv[i] = yv[i];
      }
      i2 = keepInlierA.size(0) - 1;
      len = 0;
      for (d_i = 0; d_i <= i2; d_i++) {
        if (keepInlierA[d_i]) {
          len++;
        }
      }
      r4.set_size(1, len);
      len = 0;
      for (d_i = 0; d_i <= i2; d_i++) {
        if (keepInlierA[d_i]) {
          r4[len] = d_i + 1;
          len++;
        }
      }
      inlierIndicesB.set_size(1, r4.size(1));
      i2 = r4.size(1);
      for (i = 0; i < i2; i++) {
        inlierIndicesB[i] = indices[r4[i] - 1];
      }
    }
    len = inlierIndicesB.size(0) * inlierIndicesB.size(1);
    b_locA.set_size(xv.size(0), 3);
    i2 = xv.size(0);
    for (i = 0; i < 3; i++) {
      for (p = 0; p < i2; p++) {
        b_locA[p + b_locA.size(0) * i] =
            locA.contents[(static_cast<int>(xv[p]) +
                           locA.contents.size(0) * i) -
                          1];
      }
    }
    b_ptCloudB.set_size(len, 3);
    for (i = 0; i < 3; i++) {
      for (p = 0; p < len; p++) {
        b_ptCloudB[p + b_ptCloudB.size(0) * i] =
            ptCloudB.Location[(static_cast<int>(inlierIndicesB[p]) +
                               ptCloudB.Location.size(0) * i) -
                              1];
      }
    }
    vision::internal::calibration::computeRigidTransform(b_locA, b_ptCloudB, R,
                                                         T);
    for (i = 0; i < 3; i++) {
      s = 0.0F;
      for (p = 0; p < 3; p++) {
        len = 3 * p + 9 * b_i;
        i2 = i + 3 * p;
        Rnew[i2] = (R[i] * static_cast<float>(Rs.contents[len]) +
                    R[i + 3] * static_cast<float>(Rs.contents[len + 1])) +
                   R[i + 6] * static_cast<float>(Rs.contents[len + 2]);
        s += R[i2] * static_cast<float>(Ts.contents[p + 3 * b_i]);
      }
      Tnew[i] = s + T[i];
    }
    for (i = 0; i < 3; i++) {
      p = 3 * i + 9 * (b_i + 1);
      Rs.contents[p] = Rnew[3 * i];
      Rs.contents[p + 1] = Rnew[3 * i + 1];
      Rs.contents[p + 2] = Rnew[3 * i + 2];
      Ts.contents[i + 3 * (b_i + 1)] = Tnew[i];
    }
    len = r.Location.size(0);
    b_locA.set_size(r.Location.size(0), 3);
    for (int j{0}; j < 3; j++) {
      i2 = j * len;
      for (d_i = 0; d_i < len; d_i++) {
        s = r.Location[d_i] * Rnew[j];
        s += r.Location[r.Location.size(0) + d_i] * Rnew[j + 3];
        s += r.Location[(r.Location.size(0) << 1) + d_i] * Rnew[j + 6];
        b_locA[i2 + d_i] = s;
      }
    }
    locA.contents.set_size(b_locA.size(0), 3);
    i2 = b_locA.size(0);
    for (i = 0; i < 3; i++) {
      for (p = 0; p < i2; p++) {
        locA.contents[p + locA.contents.size(0) * i] =
            b_locA[p + b_locA.size(0) * i] + Tnew[i];
      }
    }
    float fv[4];
    vision::internal::quaternion::rotationToQuaternion(Rnew, fv);
    i = 7 * (b_i + 1);
    qs.contents[i] = fv[0];
    qs.contents[i + 1] = fv[1];
    qs.contents[i + 2] = fv[2];
    qs.contents[i + 3] = fv[3];
    qs.contents[i + 4] = Tnew[0];
    qs.contents[i + 5] = Tnew[1];
    qs.contents[i + 6] = Tnew[2];
    getChangesInTransformation(&c_i, &qs, &Ts, &inlierRatio, &dT, &rdiff,
                               &tdiff);
    if ((dT <= tolerance[0]) && (inlierRatio <= tolerance[1])) {
      stopIteration = b_i + 1;
      exitg1 = true;
    } else {
      b_i++;
    }
  }
  for (i = 0; i < 3; i++) {
    len = i + 9 * stopIteration;
    b_Rs[3 * i] = Rs.contents[len];
    b_Rs[3 * i + 1] = Rs.contents[len + 3];
    b_Rs[3 * i + 2] = Rs.contents[len + 6];
  }
  svd(b_Rs, U, a__2, V);
  for (i = 0; i < 3; i++) {
    rdiff = U[i];
    inlierRatio = U[i + 3];
    dT = U[i + 6];
    for (p = 0; p < 3; p++) {
      a__2[i + 3 * p] = (rdiff * V[p] + inlierRatio * V[p + 3]) + dT * V[p + 6];
    }
  }
  for (i = 0; i < 3; i++) {
    p = i << 2;
    tform->AffineTform.T[p] = a__2[3 * i];
    tform->AffineTform.T[p + 1] = a__2[3 * i + 1];
    tform->AffineTform.T[p + 2] = a__2[3 * i + 2];
    tform->AffineTform.T[i + 12] = 0.0;
    tform->AffineTform.T[p + 3] = Ts.contents[i + 3 * stopIteration];
  }
  tform->AffineTform.T[15] = 1.0;
  rigid3d::isTransformationMatrixRigid(tform->AffineTform.T);
  for (i = 0; i < 3; i++) {
    len = i << 2;
    _2[len] = a__2[3 * i];
    _2[len + 1] = a__2[3 * i + 1];
    _2[len + 2] = a__2[3 * i + 2];
    _2[i + 12] = 0.0;
    _2[len + 3] = Ts.contents[i + 3 * stopIteration];
  }
  _2[15] = 1.0;
  rigid3d::isTransformationMatrixRigid(_2);
  tform->Data.set_size(1, 1);
  tform->Data[0] = r3;
}

} // namespace coder

// End of code generation (pcregistericp.cpp)
