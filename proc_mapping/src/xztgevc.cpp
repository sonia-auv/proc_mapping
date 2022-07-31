//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xztgevc.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "xztgevc.h"
#include "proc_mapping_data.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : const creal_T A[16]
//                creal_T V[16]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xztgevc(const creal_T A[16], creal_T V[16])
{
  creal_T work1[4];
  creal_T work2[4];
  double rworka[4];
  double anorm;
  double ascale;
  double x;
  double y;
  int d_re_tmp;
  int i;
  rworka[0] = 0.0;
  rworka[1] = 0.0;
  rworka[2] = 0.0;
  rworka[3] = 0.0;
  anorm = std::abs(A[0].re) + std::abs(A[0].im);
  for (int j{0}; j < 3; j++) {
    for (i = 0; i <= j; i++) {
      d_re_tmp = i + ((j + 1) << 2);
      rworka[j + 1] += std::abs(A[d_re_tmp].re) + std::abs(A[d_re_tmp].im);
    }
    i = (j + ((j + 1) << 2)) + 1;
    y = rworka[j + 1] + (std::abs(A[i].re) + std::abs(A[i].im));
    if (y > anorm) {
      anorm = y;
    }
  }
  x = anorm;
  if (anorm < 2.2250738585072014E-308) {
    x = 2.2250738585072014E-308;
  }
  ascale = 1.0 / x;
  for (int je{3}; je >= 0; je--) {
    double acoeff;
    double dmin;
    double salpha_im;
    double salpha_re;
    double scale;
    double temp;
    double xmx;
    int x_tmp_tmp_tmp;
    bool lscalea;
    bool lscaleb;
    x_tmp_tmp_tmp = je << 2;
    i = je + x_tmp_tmp_tmp;
    xmx = A[i].re;
    y = A[i].im;
    x = (std::abs(xmx) + std::abs(y)) * ascale;
    if (x < 1.0) {
      x = 1.0;
    }
    temp = 1.0 / x;
    salpha_re = ascale * (temp * xmx);
    salpha_im = ascale * (temp * y);
    acoeff = temp * ascale;
    if ((temp >= 2.2250738585072014E-308) &&
        (acoeff < 4.0083367200179456E-292)) {
      lscalea = true;
    } else {
      lscalea = false;
    }
    xmx = std::abs(salpha_re) + std::abs(salpha_im);
    if ((xmx >= 2.2250738585072014E-308) && (xmx < 4.0083367200179456E-292)) {
      lscaleb = true;
    } else {
      lscaleb = false;
    }
    scale = 1.0;
    if (lscalea) {
      x = anorm;
      if (anorm > 2.4948003869184E+291) {
        x = 2.4948003869184E+291;
      }
      scale = 4.0083367200179456E-292 / temp * x;
    }
    if (lscaleb) {
      y = 4.0083367200179456E-292 / xmx;
      if (y > scale) {
        scale = y;
      }
    }
    if (lscalea || lscaleb) {
      x = acoeff;
      if (acoeff < 1.0) {
        x = 1.0;
      }
      if (xmx > x) {
        x = xmx;
      }
      y = 1.0 / (2.2250738585072014E-308 * x);
      if (y < scale) {
        scale = y;
      }
      if (lscalea) {
        acoeff = ascale * (scale * temp);
      } else {
        acoeff *= scale;
      }
      salpha_re *= scale;
      salpha_im *= scale;
    }
    std::memset(&work1[0], 0, 4U * sizeof(creal_T));
    work1[je].re = 1.0;
    work1[je].im = 0.0;
    dmin = 2.2204460492503131E-16 * acoeff * anorm;
    y = 2.2204460492503131E-16 * (std::abs(salpha_re) + std::abs(salpha_im));
    if (y > dmin) {
      dmin = y;
    }
    if (dmin < 2.2250738585072014E-308) {
      dmin = 2.2250738585072014E-308;
    }
    for (int jr{0}; jr < je; jr++) {
      i = jr + x_tmp_tmp_tmp;
      work1[jr].re = acoeff * A[i].re;
      work1[jr].im = acoeff * A[i].im;
    }
    work1[je].re = 1.0;
    work1[je].im = 0.0;
    for (int j{je}; j >= 1; j--) {
      double brm;
      double d_im;
      double d_re;
      i = (j - 1) << 2;
      d_re_tmp = (j + i) - 1;
      d_re = acoeff * A[d_re_tmp].re - salpha_re;
      d_im = acoeff * A[d_re_tmp].im - salpha_im;
      if (std::abs(d_re) + std::abs(d_im) <= dmin) {
        d_re = dmin;
        d_im = 0.0;
      }
      brm = std::abs(d_re);
      y = std::abs(d_im);
      xmx = brm + y;
      if (xmx < 1.0) {
        scale = std::abs(work1[j - 1].re) + std::abs(work1[j - 1].im);
        if (scale >= 1.1235582092889474E+307 * xmx) {
          temp = 1.0 / scale;
          for (int jr{0}; jr <= je; jr++) {
            work1[jr].re *= temp;
            work1[jr].im *= temp;
          }
        }
      }
      x = work1[j - 1].re;
      temp = work1[j - 1].im;
      if (d_im == 0.0) {
        if (-temp == 0.0) {
          y = -x / d_re;
          xmx = 0.0;
        } else if (-x == 0.0) {
          y = 0.0;
          xmx = -temp / d_re;
        } else {
          y = -x / d_re;
          xmx = -temp / d_re;
        }
      } else if (d_re == 0.0) {
        if (-x == 0.0) {
          y = -temp / d_im;
          xmx = 0.0;
        } else if (-temp == 0.0) {
          y = 0.0;
          xmx = -(-x / d_im);
        } else {
          y = -temp / d_im;
          xmx = -(-x / d_im);
        }
      } else if (brm > y) {
        scale = d_im / d_re;
        xmx = d_re + scale * d_im;
        y = (-x + scale * -temp) / xmx;
        xmx = (-temp - scale * -x) / xmx;
      } else if (y == brm) {
        if (d_re > 0.0) {
          scale = 0.5;
        } else {
          scale = -0.5;
        }
        if (d_im > 0.0) {
          xmx = 0.5;
        } else {
          xmx = -0.5;
        }
        y = (-x * scale + -temp * xmx) / brm;
        xmx = (-temp * scale - -x * xmx) / brm;
      } else {
        scale = d_re / d_im;
        xmx = d_im + scale * d_re;
        y = (scale * -x + -temp) / xmx;
        xmx = (scale * -temp - (-x)) / xmx;
      }
      work1[j - 1].re = y;
      work1[j - 1].im = xmx;
      if (j > 1) {
        xmx = std::abs(y) + std::abs(xmx);
        if (xmx > 1.0) {
          temp = 1.0 / xmx;
          if (acoeff * rworka[j - 1] >= 1.1235582092889474E+307 * temp) {
            for (int jr{0}; jr <= je; jr++) {
              work1[jr].re *= temp;
              work1[jr].im *= temp;
            }
          }
        }
        d_re = acoeff * work1[j - 1].re;
        d_im = acoeff * work1[j - 1].im;
        for (int jr{0}; jr <= j - 2; jr++) {
          d_re_tmp = jr + i;
          xmx = A[d_re_tmp].im;
          x = A[d_re_tmp].re;
          work1[jr].re += d_re * x - d_im * xmx;
          work1[jr].im += d_re * xmx + d_im * x;
        }
      }
    }
    std::memset(&work2[0], 0, 4U * sizeof(creal_T));
    for (d_re_tmp = 0; d_re_tmp <= je; d_re_tmp++) {
      i = d_re_tmp << 2;
      xmx = V[i].re;
      y = work1[d_re_tmp].im;
      scale = V[i].im;
      x = work1[d_re_tmp].re;
      work2[0].re += xmx * x - scale * y;
      work2[0].im += xmx * y + scale * x;
      xmx = V[i + 1].re;
      scale = V[i + 1].im;
      work2[1].re += xmx * x - scale * y;
      work2[1].im += xmx * y + scale * x;
      xmx = V[i + 2].re;
      scale = V[i + 2].im;
      work2[2].re += xmx * x - scale * y;
      work2[2].im += xmx * y + scale * x;
      xmx = V[i + 3].re;
      scale = V[i + 3].im;
      work2[3].re += xmx * x - scale * y;
      work2[3].im += xmx * y + scale * x;
    }
    xmx = std::abs(work2[0].re) + std::abs(work2[0].im);
    y = std::abs(work2[1].re) + std::abs(work2[1].im);
    if (y > xmx) {
      xmx = y;
    }
    y = std::abs(work2[2].re) + std::abs(work2[2].im);
    if (y > xmx) {
      xmx = y;
    }
    y = std::abs(work2[3].re) + std::abs(work2[3].im);
    if (y > xmx) {
      xmx = y;
    }
    if (xmx > 2.2250738585072014E-308) {
      temp = 1.0 / xmx;
      V[x_tmp_tmp_tmp].re = temp * work2[0].re;
      V[x_tmp_tmp_tmp].im = temp * work2[0].im;
      V[x_tmp_tmp_tmp + 1].re = temp * work2[1].re;
      V[x_tmp_tmp_tmp + 1].im = temp * work2[1].im;
      V[x_tmp_tmp_tmp + 2].re = temp * work2[2].re;
      V[x_tmp_tmp_tmp + 2].im = temp * work2[2].im;
      V[x_tmp_tmp_tmp + 3].re = temp * work2[3].re;
      V[x_tmp_tmp_tmp + 3].im = temp * work2[3].im;
    } else {
      V[x_tmp_tmp_tmp].re = 0.0;
      V[x_tmp_tmp_tmp].im = 0.0;
      V[x_tmp_tmp_tmp + 1].re = 0.0;
      V[x_tmp_tmp_tmp + 1].im = 0.0;
      V[x_tmp_tmp_tmp + 2].re = 0.0;
      V[x_tmp_tmp_tmp + 2].im = 0.0;
      i = (je << 2) + 3;
      V[i].re = 0.0;
      V[i].im = 0.0;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xztgevc.cpp
//
// [EOF]
//
