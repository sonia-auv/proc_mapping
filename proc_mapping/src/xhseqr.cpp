//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xhseqr.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "xhseqr.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xnrm2.h"
#include "xrot.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : double h[16]
//                double z[16]
// Return Type  : int
//
namespace coder {
namespace internal {
namespace lapack {
int xhseqr(double h[16], double z[16])
{
  double v[3];
  double aa;
  double ab;
  double ba;
  double bb;
  double rt1r;
  double rt2r;
  double s;
  double s_tmp;
  double tst;
  int i;
  int info;
  bool exitg1;
  info = 0;
  v[0] = 0.0;
  v[1] = 0.0;
  v[2] = 0.0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  i = 3;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    int L;
    int b_i;
    int b_k;
    int hoffset;
    int its;
    int knt;
    bool exitg2;
    bool goto150;
    L = 1;
    goto150 = false;
    its = 0;
    exitg2 = false;
    while ((!exitg2) && (its < 301)) {
      int k;
      bool exitg3;
      k = i;
      exitg3 = false;
      while ((!exitg3) && (k + 1 > L)) {
        b_i = k + ((k - 1) << 2);
        ba = std::abs(h[b_i]);
        if (ba <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          hoffset = k + (k << 2);
          bb = std::abs(h[hoffset]);
          tst = std::abs(h[b_i - 1]) + bb;
          if (tst == 0.0) {
            if (k - 1 >= 1) {
              tst = std::abs(h[(k + ((k - 2) << 2)) - 1]);
            }
            if (k + 2 <= 4) {
              tst += std::abs(h[hoffset + 1]);
            }
          }
          if (ba <= 2.2204460492503131E-16 * tst) {
            tst = std::abs(h[hoffset - 1]);
            if (ba > tst) {
              ab = ba;
              ba = tst;
            } else {
              ab = tst;
            }
            tst = std::abs(h[b_i - 1] - h[hoffset]);
            if (bb > tst) {
              aa = bb;
              bb = tst;
            } else {
              aa = tst;
            }
            s = aa + ab;
            if (ba * (ab / s) <=
                std::fmax(4.0083367200179456E-292,
                          2.2204460492503131E-16 * (bb * (aa / s)))) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }
      }
      L = k + 1;
      if (k + 1 > 1) {
        h[k + ((k - 1) << 2)] = 0.0;
      }
      if (k + 1 >= i) {
        goto150 = true;
        exitg2 = true;
      } else {
        int m;
        if (its == 10) {
          hoffset = k + (k << 2);
          s = std::abs(h[hoffset + 1]) + std::abs(h[(k + ((k + 1) << 2)) + 2]);
          tst = 0.75 * s + h[hoffset];
          aa = -0.4375 * s;
          ab = s;
          bb = tst;
        } else if (its == 20) {
          s = std::abs(h[i + ((i - 1) << 2)]) +
              std::abs(h[(i + ((i - 2) << 2)) - 1]);
          tst = 0.75 * s + h[i + (i << 2)];
          aa = -0.4375 * s;
          ab = s;
          bb = tst;
        } else {
          hoffset = i + ((i - 1) << 2);
          tst = h[hoffset - 1];
          ab = h[hoffset];
          aa = h[(i + (i << 2)) - 1];
          bb = h[i + (i << 2)];
        }
        s = ((std::abs(tst) + std::abs(aa)) + std::abs(ab)) + std::abs(bb);
        if (s == 0.0) {
          rt1r = 0.0;
          ba = 0.0;
          rt2r = 0.0;
          bb = 0.0;
        } else {
          tst /= s;
          ab /= s;
          aa /= s;
          bb /= s;
          ba = (tst + bb) / 2.0;
          tst = (tst - ba) * (bb - ba) - aa * ab;
          ab = std::sqrt(std::abs(tst));
          if (tst >= 0.0) {
            rt1r = ba * s;
            rt2r = rt1r;
            ba = ab * s;
            bb = -ba;
          } else {
            rt1r = ba + ab;
            rt2r = ba - ab;
            if (std::abs(rt1r - bb) <= std::abs(rt2r - bb)) {
              rt1r *= s;
              rt2r = rt1r;
            } else {
              rt2r *= s;
              rt1r = rt2r;
            }
            ba = 0.0;
            bb = 0.0;
          }
        }
        m = i - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          hoffset = m + ((m - 1) << 2);
          tst = h[hoffset];
          s_tmp = h[hoffset - 1];
          ab = s_tmp - rt2r;
          s = (std::abs(ab) + std::abs(bb)) + std::abs(tst);
          aa = tst / s;
          hoffset = m + (m << 2);
          v[0] =
              (aa * h[hoffset - 1] + (s_tmp - rt1r) * (ab / s)) - ba * (bb / s);
          tst = h[hoffset];
          v[1] = aa * (((s_tmp + tst) - rt1r) - rt2r);
          v[2] = aa * h[hoffset + 1];
          s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
          v[0] /= s;
          v[1] /= s;
          v[2] /= s;
          if (m == k + 1) {
            exitg3 = true;
          } else {
            b_i = m + ((m - 2) << 2);
            if (std::abs(h[b_i - 1]) * (std::abs(v[1]) + std::abs(v[2])) <=
                2.2204460492503131E-16 * std::abs(v[0]) *
                    ((std::abs(h[b_i - 2]) + std::abs(s_tmp)) +
                     std::abs(tst))) {
              exitg3 = true;
            } else {
              m--;
            }
          }
        }
        for (int c_k{m}; c_k <= i; c_k++) {
          int nr;
          hoffset = (i - c_k) + 2;
          if (hoffset >= 3) {
            nr = 3;
          } else {
            nr = hoffset;
          }
          if (c_k > m) {
            hoffset = (c_k + ((c_k - 2) << 2)) - 1;
            for (int j{0}; j < nr; j++) {
              v[j] = h[j + hoffset];
            }
          }
          ab = v[0];
          bb = 0.0;
          if (nr > 0) {
            tst = blas::xnrm2(nr - 1, v);
            if (tst != 0.0) {
              aa = rt_hypotd_snf(v[0], tst);
              if (v[0] >= 0.0) {
                aa = -aa;
              }
              if (std::abs(aa) < 1.0020841800044864E-292) {
                knt = 0;
                do {
                  knt++;
                  for (b_k = 2; b_k <= nr; b_k++) {
                    v[b_k - 1] *= 9.9792015476736E+291;
                  }
                  aa *= 9.9792015476736E+291;
                  ab *= 9.9792015476736E+291;
                } while ((std::abs(aa) < 1.0020841800044864E-292) &&
                         (knt < 20));
                aa = rt_hypotd_snf(ab, blas::xnrm2(nr - 1, v));
                if (ab >= 0.0) {
                  aa = -aa;
                }
                bb = (aa - ab) / aa;
                tst = 1.0 / (ab - aa);
                for (b_k = 2; b_k <= nr; b_k++) {
                  v[b_k - 1] *= tst;
                }
                for (b_k = 0; b_k < knt; b_k++) {
                  aa *= 1.0020841800044864E-292;
                }
                ab = aa;
              } else {
                bb = (aa - v[0]) / aa;
                tst = 1.0 / (v[0] - aa);
                for (b_k = 2; b_k <= nr; b_k++) {
                  v[b_k - 1] *= tst;
                }
                ab = aa;
              }
            }
          }
          v[0] = ab;
          if (c_k > m) {
            h[(c_k + ((c_k - 2) << 2)) - 1] = ab;
            b_i = c_k + ((c_k - 2) << 2);
            h[b_i] = 0.0;
            if (c_k < i) {
              h[b_i + 1] = 0.0;
            }
          } else if (m > k + 1) {
            h[(c_k + ((c_k - 2) << 2)) - 1] *= 1.0 - bb;
          }
          rt1r = v[1];
          tst = bb * v[1];
          if (nr == 3) {
            s = v[2];
            ba = bb * v[2];
            for (int j{c_k}; j < 5; j++) {
              hoffset = c_k + ((j - 1) << 2);
              aa = (h[hoffset - 1] + rt1r * h[hoffset]) + s * h[hoffset + 1];
              h[hoffset - 1] -= aa * bb;
              h[hoffset] -= aa * tst;
              h[hoffset + 1] -= aa * ba;
            }
            if (c_k + 3 <= i + 1) {
              b_i = c_k + 2;
            } else {
              b_i = i;
            }
            for (int j{0}; j <= b_i; j++) {
              hoffset = j + ((c_k - 1) << 2);
              knt = j + (c_k << 2);
              b_k = j + ((c_k + 1) << 2);
              aa = (h[hoffset] + rt1r * h[knt]) + s * h[b_k];
              h[hoffset] -= aa * bb;
              h[knt] -= aa * tst;
              h[b_k] -= aa * ba;
            }
            for (int j{0}; j < 4; j++) {
              hoffset = j + ((c_k - 1) << 2);
              ab = z[hoffset];
              knt = j + (c_k << 2);
              b_k = j + ((c_k + 1) << 2);
              aa = (ab + rt1r * z[knt]) + s * z[b_k];
              z[hoffset] = ab - aa * bb;
              z[knt] -= aa * tst;
              z[b_k] -= aa * ba;
            }
          } else if (nr == 2) {
            for (int j{c_k}; j < 5; j++) {
              hoffset = c_k + ((j - 1) << 2);
              ab = h[hoffset - 1];
              aa = ab + rt1r * h[hoffset];
              h[hoffset - 1] = ab - aa * bb;
              h[hoffset] -= aa * tst;
            }
            for (int j{0}; j <= i; j++) {
              hoffset = j + ((c_k - 1) << 2);
              knt = j + (c_k << 2);
              aa = h[hoffset] + rt1r * h[knt];
              h[hoffset] -= aa * bb;
              h[knt] -= aa * tst;
            }
            for (int j{0}; j < 4; j++) {
              hoffset = j + ((c_k - 1) << 2);
              ab = z[hoffset];
              knt = j + (c_k << 2);
              aa = ab + rt1r * z[knt];
              z[hoffset] = ab - aa * bb;
              z[knt] -= aa * tst;
            }
          }
        }
        its++;
      }
    }
    if (!goto150) {
      info = i + 1;
      exitg1 = true;
    } else {
      if ((L != i + 1) && (L == i)) {
        knt = i << 2;
        b_i = i + knt;
        rt1r = h[b_i - 1];
        b_k = (i - 1) << 2;
        hoffset = i + b_k;
        s = h[hoffset];
        tst = h[b_i];
        reflapack::xdlanv2(&h[(i + ((i - 1) << 2)) - 1], &rt1r, &s, &tst, &ab,
                           &aa, &ba, &bb, &s_tmp, &rt2r);
        h[b_i - 1] = rt1r;
        h[hoffset] = s;
        h[b_i] = tst;
        if (i + 1 < 4) {
          hoffset = (i + 1) << 2;
          blas::xrot(3 - i, h, i + hoffset, (i + hoffset) + 1, s_tmp, rt2r);
        }
        blas::b_xrot(i - 1, h, b_k + 1, knt + 1, s_tmp, rt2r);
        tst = s_tmp * z[b_k] + rt2r * z[knt];
        z[knt] = s_tmp * z[knt] - rt2r * z[b_k];
        z[b_k] = tst;
        tst = z[knt + 1];
        ab = z[b_k + 1];
        z[knt + 1] = s_tmp * tst - rt2r * ab;
        z[b_k + 1] = s_tmp * ab + rt2r * tst;
        tst = z[knt + 2];
        ab = z[b_k + 2];
        z[knt + 2] = s_tmp * tst - rt2r * ab;
        z[b_k + 2] = s_tmp * ab + rt2r * tst;
        tst = z[knt + 3];
        ab = z[b_k + 3];
        z[knt + 3] = s_tmp * tst - rt2r * ab;
        z[b_k + 3] = s_tmp * ab + rt2r * tst;
      }
      i = L - 2;
    }
  }
  h[3] = 0.0;
  return info;
}

} // namespace lapack
} // namespace internal
} // namespace coder

//
// File trailer for xhseqr.cpp
//
// [EOF]
//
