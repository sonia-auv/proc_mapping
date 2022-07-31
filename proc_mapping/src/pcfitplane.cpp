//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcfitplane.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "pcfitplane.h"
#include "Kdtree.h"
#include "blockedSummation.h"
#include "dot.h"
#include "find.h"
#include "msac.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &model
//                const ::coder::array<double, 2U> &points
//                ::coder::array<double, 1U> &dis
// Return Type  : void
//
namespace coder {
void evalPlane(const ::coder::array<double, 2U> &model,
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

//
// Arguments    : const pointCloud *varargin_1
//                double varargin_2
//                planeModel *iobj_0
//                planeModel **model
//                ::coder::array<double, 1U> &inlierIndices
//                ::coder::array<double, 1U> &outlierIndices
//                ::coder::array<double, 2U> &meanError
// Return Type  : void
//
void pcfitplane(const pointCloud *varargin_1, double varargin_2,
                planeModel *iobj_0, planeModel **model,
                ::coder::array<double, 1U> &inlierIndices,
                ::coder::array<double, 1U> &outlierIndices,
                ::coder::array<double, 2U> &meanError)
{
  ::coder::array<double, 2U> *points;
  pointCloud pc;
  pointCloud *b_pc;
  vision::internal::codegen::Kdtree lobj_1;
  array<double, 2U> a;
  array<double, 2U> denorm;
  array<double, 2U> location;
  array<double, 2U> modelParams;
  array<double, 2U> normAxis;
  array<double, 1U> distances;
  array<double, 1U> intensity;
  array<double, 1U> y;
  array<int, 1U> r;
  array<int, 1U> r1;
  array<unsigned char, 2U> color;
  array<signed char, 2U> params_referenceVector;
  array<bool, 2U> x;
  array<bool, 1U> flag;
  int i;
  int k;
  int nx;
  int status;
  bool isFound;
  pc.matlabCodegenIsDeleted = true;
  params_referenceVector.set_size(1, 3);
  params_referenceVector[0] = 1;
  params_referenceVector[1] = 0;
  params_referenceVector[2] = 0;
  varargin_1->extractValidPoints(location, color, denorm, intensity, a, flag);
  b_pc = pc.init(location, color, denorm, intensity, &lobj_1);
  b_pc->RangeData.set_size(a.size(0), a.size(1));
  nx = a.size(0) * a.size(1);
  for (i = 0; i < nx; i++) {
    b_pc->RangeData[i] = a[i];
  }
  b_eml_find(flag, r);
  intensity.set_size(r.size(0));
  nx = r.size(0);
  for (i = 0; i < nx; i++) {
    intensity[i] = r[i];
  }
  nx = b_pc->Location.size(0);
  status = (nx < 3);
  modelParams.set_size(1, 4);
  modelParams[0] = 0.0;
  modelParams[1] = 0.0;
  modelParams[2] = 0.0;
  modelParams[3] = 0.0;
  if (status == 0) {
    denorm.set_size(1, 1);
    denorm[0] = (params_referenceVector[0] * params_referenceVector[0] +
                 params_referenceVector[1] * params_referenceVector[1]) +
                params_referenceVector[2] * params_referenceVector[2];
    denorm[0] = std::sqrt(denorm[0]);
    normAxis.set_size(1, 3);
    normAxis[0] = denorm[0];
    normAxis[1] = denorm[0];
    normAxis[2] = denorm[0];
    normAxis.set_size(1, 3);
    for (i = 0; i < 3; i++) {
      normAxis[i] =
          static_cast<double>(params_referenceVector[i]) / normAxis[i];
    }
    location.set_size(pc.Location.size(0), 3);
    nx = pc.Location.size(0) * pc.Location.size(1) - 1;
    for (i = 0; i <= nx; i++) {
      location[i] = pc.Location[i];
    }
    vision::internal::ransac::msac(location, varargin_2, normAxis, &isFound,
                                   modelParams);
    if (isFound) {
      double dv[3];
      double b_varargin_2;
      dv[0] = (*(double(*)[3]) & modelParams[0])[0];
      dv[1] = (*(double(*)[3]) & modelParams[0])[1];
      dv[2] = (*(double(*)[3]) & modelParams[0])[2];
      dot(normAxis, dv, a);
      nx = a.size(0) * a.size(1);
      for (i = 0; i < nx; i++) {
        b_varargin_2 = a[i];
        a[i] = std::fmax(-1.0, b_varargin_2);
      }
      nx = a.size(0) * a.size(1);
      for (i = 0; i < nx; i++) {
        b_varargin_2 = a[i];
        a[i] = std::fmin(1.0, b_varargin_2);
      }
      nx = a.size(0) * a.size(1);
      for (k = 0; k < nx; k++) {
        a[k] = std::acos(a[k]);
      }
      nx = a.size(0) * a.size(1);
      denorm.set_size(a.size(0), a.size(1));
      for (k = 0; k < nx; k++) {
        denorm[k] = std::abs(a[k]);
      }
      x.set_size(denorm.size(0), denorm.size(1));
      nx = denorm.size(0) * denorm.size(1);
      for (i = 0; i < nx; i++) {
        x[i] = (denorm[i] > 1.5707963267948966);
      }
      isFound = ((x.size(0) != 0) && (x.size(1) != 0));
      if (isFound) {
        bool exitg1;
        i = x.size(0) * x.size(1);
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k <= i - 1)) {
          if (!x[k]) {
            isFound = false;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }
      if (isFound) {
        nx = modelParams.size(0) * modelParams.size(1);
        for (i = 0; i < nx; i++) {
          modelParams[i] = -modelParams[i];
        }
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
  iobj_0->Parameters.set_size(modelParams.size(0), modelParams.size(1));
  nx = modelParams.size(0) * modelParams.size(1);
  for (i = 0; i < nx; i++) {
    iobj_0->Parameters[i] = modelParams[i];
  }
  if (status == 0) {
    points = &pc.Location;
    nx = points->size(0);
    y.set_size(points->size(0));
    for (k = 0; k < nx; k++) {
      y[k] = ((*points)[k] * modelParams[0] +
              (*points)[points->size(0) + k] * modelParams[1]) +
             (*points)[(points->size(0) << 1) + k] * modelParams[2];
    }
    nx = y.size(0);
    for (i = 0; i < nx; i++) {
      y[i] = y[i] + modelParams[3];
    }
    nx = y.size(0);
    distances.set_size(y.size(0));
    for (k = 0; k < nx; k++) {
      distances[k] = std::abs(y[k]);
    }
    x.set_size(distances.size(0), 1);
    nx = distances.size(0);
    for (i = 0; i < nx; i++) {
      x[i] = (distances[i] < varargin_2);
    }
    status = x.size(0) - 1;
    nx = 0;
    for (k = 0; k <= status; k++) {
      if (x[k]) {
        nx++;
      }
    }
    inlierIndices.set_size(nx);
    nx = 0;
    for (k = 0; k <= status; k++) {
      if (x[k]) {
        inlierIndices[nx] = intensity[k];
        nx++;
      }
    }
    nx = varargin_1->Location.size(0);
    flag.set_size(nx);
    for (i = 0; i < nx; i++) {
      flag[i] = true;
    }
    r.set_size(inlierIndices.size(0));
    nx = inlierIndices.size(0);
    for (i = 0; i < nx; i++) {
      r[i] = static_cast<int>(inlierIndices[i]);
    }
    nx = r.size(0);
    for (i = 0; i < nx; i++) {
      flag[r[i] - 1] = false;
    }
    b_eml_find(flag, r);
    outlierIndices.set_size(r.size(0));
    nx = r.size(0);
    for (i = 0; i < nx; i++) {
      outlierIndices[i] = r[i];
    }
    x.set_size(distances.size(0), 1);
    nx = distances.size(0);
    for (i = 0; i < nx; i++) {
      x[i] = (distances[i] < varargin_2);
    }
    status = x.size(0) - 1;
    nx = 0;
    for (k = 0; k <= status; k++) {
      if (x[k]) {
        nx++;
      }
    }
    r1.set_size(nx);
    nx = 0;
    for (k = 0; k <= status; k++) {
      if (x[k]) {
        r1[nx] = k + 1;
        nx++;
      }
    }
    intensity.set_size(r1.size(0));
    nx = r1.size(0);
    for (i = 0; i < nx; i++) {
      intensity[i] = distances[r1[i] - 1];
    }
    meanError.set_size(1, 1);
    meanError[0] = blockedSummation(intensity, r1.size(0)) /
                   static_cast<double>(r1.size(0));
  } else {
    inlierIndices.set_size(0);
    outlierIndices.set_size(0);
    meanError.set_size(0, 0);
  }
}

} // namespace coder

//
// File trailer for pcfitplane.cpp
//
// [EOF]
//
