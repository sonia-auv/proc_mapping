//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: histcounts.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "histcounts.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <math.h>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 1U> &x
//                double varargin_1
//                ::coder::array<double, 2U> &n
//                ::coder::array<double, 2U> &edges
// Return Type  : void
//
namespace coder {
void histcounts(const ::coder::array<double, 1U> &x, double varargin_1,
                ::coder::array<double, 2U> &n,
                ::coder::array<double, 2U> &edges)
{
  array<int, 2U> ni;
  double HighLimit;
  double LowLimit;
  double epsxScale;
  double leftEdge;
  double xScale;
  int i;
  int k;
  int low_i;
  int low_ip1;
  int mid_i;
  int nbinsActual;
  int nx;
  low_ip1 = static_cast<int>(varargin_1);
  nx = x.size(0);
  k = 0;
  while ((k + 1 <= nx) && (std::isinf(x[k]) || std::isnan(x[k]))) {
    k++;
  }
  if (k + 1 > x.size(0)) {
    LowLimit = 0.0;
    nbinsActual = 0;
  } else {
    LowLimit = x[k];
    nbinsActual = 1;
  }
  HighLimit = LowLimit;
  i = k + 2;
  for (k = i; k <= nx; k++) {
    xScale = x[k - 1];
    if ((!std::isinf(xScale)) && (!std::isnan(xScale))) {
      if (xScale < LowLimit) {
        LowLimit = xScale;
      } else if (xScale > HighLimit) {
        HighLimit = xScale;
      }
      nbinsActual++;
    }
  }
  if (nbinsActual > 0) {
    double delta1;
    bool b;
    bool b1;
    xScale = std::fmax(std::abs(LowLimit), std::abs(HighLimit));
    b = !std::isinf(xScale);
    b1 = !std::isnan(xScale);
    if (b && b1) {
      if (xScale <= 2.2250738585072014E-308) {
        epsxScale = 4.94065645841247E-324;
      } else {
        frexp(xScale, &low_i);
        epsxScale = std::ldexp(1.0, low_i - 53);
      }
    } else {
      epsxScale = rtNaN;
    }
    delta1 = HighLimit - LowLimit;
    leftEdge = std::fmax(
        delta1 / static_cast<double>(static_cast<int>(varargin_1)), epsxScale);
    if (delta1 > std::fmax(std::sqrt(epsxScale), 2.2250738585072014E-308)) {
      xScale = rt_powd_snf(10.0, std::floor(std::log10(leftEdge)));
      xScale *= std::floor(leftEdge / xScale);
      leftEdge =
          std::fmax(std::fmin(xScale * std::floor(LowLimit / xScale), LowLimit),
                    -1.7976931348623157E+308);
      nbinsActual = static_cast<int>(varargin_1) + 1;
      if (static_cast<int>(varargin_1) > 1) {
        xScale = (HighLimit - leftEdge) /
                 static_cast<double>(static_cast<int>(varargin_1));
        epsxScale = rt_powd_snf(
            10.0,
            std::floor(std::log10(
                (HighLimit - leftEdge) /
                    (static_cast<double>(static_cast<int>(varargin_1)) - 1.0) -
                xScale)));
        xScale = epsxScale * std::ceil(xScale / epsxScale);
      }
      epsxScale =
          std::fmin(std::fmax(leftEdge + static_cast<double>(
                                             static_cast<int>(varargin_1)) *
                                             xScale,
                              HighLimit),
                    1.7976931348623157E+308);
    } else {
      nbinsActual = static_cast<int>(varargin_1) + 1;
      if (b && b1) {
        if (xScale <= 2.2250738585072014E-308) {
          xScale = 4.94065645841247E-324;
        } else {
          frexp(xScale, &mid_i);
          xScale = std::ldexp(1.0, mid_i - 53);
        }
      } else {
        xScale = rtNaN;
      }
      xScale = std::fmax(
          1.0, std::ceil(static_cast<double>(static_cast<int>(varargin_1)) *
                         xScale));
      leftEdge = std::floor(2.0 * (LowLimit - xScale / 4.0)) / 2.0;
      epsxScale = std::ceil(2.0 * (HighLimit + xScale / 4.0)) / 2.0;
      xScale = (epsxScale - leftEdge) /
               static_cast<double>(static_cast<int>(varargin_1));
    }
    if ((!std::isinf(xScale)) && (!std::isnan(xScale))) {
      edges.set_size(1, nbinsActual);
      for (i = 0; i < nbinsActual; i++) {
        edges[i] = 0.0;
      }
      edges[0] = leftEdge;
      for (low_i = 0; low_i <= nbinsActual - 3; low_i++) {
        edges[low_i + 1] =
            leftEdge + (static_cast<double>(low_i) + 1.0) * xScale;
      }
      edges[edges.size(1) - 1] = epsxScale;
    } else if (!(static_cast<double>(nbinsActual - 1) + 1.0 >= 0.0)) {
      edges.set_size(1, 0);
    } else {
      edges.set_size(1, nbinsActual);
      if (nbinsActual >= 1) {
        edges[nbinsActual - 1] = epsxScale;
        if (edges.size(1) >= 2) {
          edges[0] = leftEdge;
          if (edges.size(1) >= 3) {
            if ((leftEdge == -epsxScale) && (nbinsActual > 2)) {
              xScale = epsxScale / (static_cast<double>(nbinsActual) - 1.0);
              i = nbinsActual - 1;
              for (k = 2; k <= i; k++) {
                edges[k - 1] =
                    static_cast<double>(((k << 1) - nbinsActual) - 1) * xScale;
              }
              if ((nbinsActual & 1) == 1) {
                edges[nbinsActual >> 1] = 0.0;
              }
            } else if (((leftEdge < 0.0) != (epsxScale < 0.0)) &&
                       ((std::abs(leftEdge) > 8.9884656743115785E+307) ||
                        (std::abs(epsxScale) > 8.9884656743115785E+307))) {
              delta1 = leftEdge / (static_cast<double>(edges.size(1)) - 1.0);
              xScale = epsxScale / (static_cast<double>(edges.size(1)) - 1.0);
              i = edges.size(1);
              for (k = 0; k <= i - 3; k++) {
                edges[k + 1] =
                    (leftEdge + xScale * (static_cast<double>(k) + 1.0)) -
                    delta1 * (static_cast<double>(k) + 1.0);
              }
            } else {
              delta1 = (epsxScale - leftEdge) /
                       (static_cast<double>(edges.size(1)) - 1.0);
              i = edges.size(1);
              for (k = 0; k <= i - 3; k++) {
                edges[k + 1] =
                    leftEdge + (static_cast<double>(k) + 1.0) * delta1;
              }
            }
          }
        }
      }
    }
  } else if (static_cast<int>(varargin_1) >= 2) {
    nbinsActual = static_cast<int>(varargin_1) + 1;
    edges.set_size(1, nbinsActual);
    for (i = 0; i < nbinsActual; i++) {
      edges[i] = 0.0;
    }
    for (k = 0; k <= low_ip1; k++) {
      edges[k] = k;
    }
  } else {
    edges.set_size(1, 2);
    edges[0] = 0.0;
    edges[1] = 1.0;
  }
  ni.set_size(1, edges.size(1) - 1);
  nbinsActual = edges.size(1) - 1;
  for (i = 0; i < nbinsActual; i++) {
    ni[i] = 0;
  }
  nx = x.size(0);
  leftEdge = edges[0];
  xScale = edges[1] - edges[0];
  for (k = 0; k < nx; k++) {
    if ((x[k] >= leftEdge) && (x[k] <= edges[edges.size(1) - 1])) {
      epsxScale = std::ceil((x[k] - leftEdge) / xScale);
      if ((epsxScale >= 1.0) && (epsxScale < edges.size(1)) &&
          (x[k] >= edges[static_cast<int>(epsxScale) - 1]) &&
          (x[k] < edges[static_cast<int>(epsxScale)])) {
        ni[static_cast<int>(epsxScale) - 1] =
            ni[static_cast<int>(epsxScale) - 1] + 1;
      } else {
        nbinsActual = edges.size(1);
        low_i = 1;
        low_ip1 = 2;
        while (nbinsActual > low_ip1) {
          mid_i = (low_i >> 1) + (nbinsActual >> 1);
          if (((low_i & 1) == 1) && ((nbinsActual & 1) == 1)) {
            mid_i++;
          }
          if (x[k] >= edges[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            nbinsActual = mid_i;
          }
        }
        ni[low_i - 1] = ni[low_i - 1] + 1;
      }
    }
  }
  n.set_size(1, ni.size(1));
  nbinsActual = ni.size(1);
  for (i = 0; i < nbinsActual; i++) {
    n[i] = ni[i];
  }
}

} // namespace coder

//
// File trailer for histcounts.cpp
//
// [EOF]
//
