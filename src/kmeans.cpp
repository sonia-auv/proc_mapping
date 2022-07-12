//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// kmeans.cpp
//
// Code generation for function 'kmeans'
//

// Include files
#include "kmeans.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Declarations
namespace coder {
static void batchUpdate(const ::coder::array<double, 2U> &X,
                        ::coder::array<int, 1U> &idx, double C[6],
                        ::coder::array<double, 2U> &D, int counts[2],
                        bool *converged, int *iter);

static void gcentroids(double C[6], int counts[2],
                       const ::coder::array<double, 2U> &X,
                       const ::coder::array<int, 1U> &idx,
                       const int clusters[2], int nclusters);

} // namespace coder

// Function Definitions
namespace coder {
static void batchUpdate(const ::coder::array<double, 2U> &X,
                        ::coder::array<int, 1U> &idx, double C[6],
                        ::coder::array<double, 2U> &D, int counts[2],
                        bool *converged, int *iter)
{
  array<double, 1U> d;
  array<int, 1U> moved;
  array<int, 1U> previdx;
  array<signed char, 1U> nidx;
  array<bool, 1U> b;
  double prevtotsumD;
  int changed[2];
  int empties[2];
  int cr;
  int i;
  int n;
  int nchanged;
  n = X.size(0) - 1;
  empties[0] = 0;
  empties[1] = 0;
  previdx.set_size(X.size(0));
  cr = X.size(0);
  for (i = 0; i < cr; i++) {
    previdx[i] = 0;
  }
  moved.set_size(X.size(0));
  cr = X.size(0);
  for (i = 0; i < cr; i++) {
    moved[i] = 0;
  }
  changed[0] = 1;
  changed[1] = 2;
  nchanged = 1;
  prevtotsumD = rtInf;
  *iter = 0;
  *converged = false;
  int b_n;
  int exitg1;
  do {
    double maxd;
    int cc;
    int j;
    int nempty;
    exitg1 = 0;
    (*iter)++;
    gcentroids(C, counts, X, idx, changed, nchanged + 1);
    b_n = X.size(0) - 1;
    for (int b_i{0}; b_i <= nchanged; b_i++) {
      cr = changed[b_i] - 1;
      for (cc = 0; cc <= b_n; cc++) {
        maxd = X[cc] - C[cr];
        D[cc + D.size(0) * cr] = maxd * maxd;
      }
      for (j = 0; j < 2; j++) {
        for (cc = 0; cc <= b_n; cc++) {
          maxd = X[cc + X.size(0) * (j + 1)] - C[cr + ((j + 1) << 1)];
          D[cc + D.size(0) * cr] = D[cc + D.size(0) * cr] + maxd * maxd;
        }
      }
    }
    nempty = -1;
    for (j = 0; j <= nchanged; j++) {
      if (counts[changed[j] - 1] == 0) {
        nempty++;
        empties[nempty] = changed[j];
      }
    }
    if (nempty + 1 > 0) {
      int c_n;
      int d_n;
      b_n = X.size(0) - 1;
      c_n = X.size(0);
      d_n = X.size(0) - 1;
      for (int b_i{0}; b_i <= nempty; b_i++) {
        int from;
        bool exitg2;
        d.set_size(n + 1);
        d[0] = D[D.size(0) * (idx[0] - 1)];
        maxd = d[0];
        cr = 0;
        for (j = 0; j <= n; j++) {
          double b_d;
          b_d = D[j + D.size(0) * (idx[j] - 1)];
          if (b_d > maxd) {
            maxd = b_d;
            cr = j;
          }
        }
        from = idx[cr] - 1;
        if (counts[idx[cr] - 1] < 2) {
          j = 0;
          exitg2 = false;
          while ((!exitg2) && (j <= n)) {
            if (counts[j] > 1) {
              from = j;
              exitg2 = true;
            } else {
              j++;
            }
          }
          j = 0;
          exitg2 = false;
          while ((!exitg2) && (j <= n)) {
            if (idx[j] == from + 1) {
              cr = j;
              exitg2 = true;
            } else {
              j++;
            }
          }
        }
        C[empties[b_i] - 1] = X[cr];
        C[empties[b_i] + 1] = X[cr + X.size(0)];
        C[empties[b_i] + 3] = X[cr + X.size(0) * 2];
        counts[empties[b_i] - 1] = 1;
        i = empties[b_i];
        idx[cr] = i;
        for (cc = 0; cc <= b_n; cc++) {
          maxd = X[cc] - C[i - 1];
          D[cc + D.size(0) * (i - 1)] = maxd * maxd;
        }
        for (j = 0; j < 2; j++) {
          for (cc = 0; cc <= b_n; cc++) {
            maxd = X[cc + X.size(0) * (j + 1)] - C[(i + ((j + 1) << 1)) - 1];
            D[cc + D.size(0) * (i - 1)] =
                D[cc + D.size(0) * (i - 1)] + maxd * maxd;
          }
        }
        counts[from] = 0;
        C[from] = rtNaN;
        C[from + 2] = rtNaN;
        C[from + 4] = rtNaN;
        cc = 0;
        C[from] = 0.0;
        C[from + 2] = 0.0;
        C[from + 4] = 0.0;
        for (cr = 0; cr < c_n; cr++) {
          if (idx[cr] == from + 1) {
            cc++;
            C[from] += X[cr];
            C[from + 2] += X[cr + X.size(0)];
            C[from + 4] += X[cr + X.size(0) * 2];
          }
        }
        counts[from] = cc;
        C[from] /= static_cast<double>(cc);
        C[from + 2] /= static_cast<double>(cc);
        C[from + 4] /= static_cast<double>(cc);
        for (cc = 0; cc <= d_n; cc++) {
          maxd = X[cc] - C[from];
          D[cc + D.size(0) * from] = maxd * maxd;
        }
        for (j = 0; j < 2; j++) {
          for (cc = 0; cc <= d_n; cc++) {
            maxd = X[cc + X.size(0) * (j + 1)] - C[from + ((j + 1) << 1)];
            D[cc + D.size(0) * from] = D[cc + D.size(0) * from] + maxd * maxd;
          }
        }
        if (nchanged + 1 < 2) {
          j = 0;
          exitg2 = false;
          while ((!exitg2) && ((j <= nchanged) && (from + 1 != changed[0]))) {
            if (from + 1 > changed[0]) {
              if (nchanged + 1 >= 1) {
                changed[1] = 1;
              }
              changed[0] = 2;
              nchanged++;
              exitg2 = true;
            } else {
              j = 1;
            }
          }
        }
      }
    }
    maxd = 0.0;
    for (int b_i{0}; b_i <= n; b_i++) {
      maxd += D[b_i + D.size(0) * (idx[b_i] - 1)];
    }
    if (prevtotsumD <= maxd) {
      idx.set_size(previdx.size(0));
      cr = previdx.size(0);
      for (i = 0; i < cr; i++) {
        idx[i] = previdx[i];
      }
      gcentroids(C, counts, X, previdx, changed, nchanged + 1);
      (*iter)--;
      exitg1 = 1;
    } else if (*iter >= 100) {
      exitg1 = 1;
    } else {
      previdx.set_size(idx.size(0));
      cr = idx.size(0);
      for (i = 0; i < cr; i++) {
        previdx[i] = idx[i];
      }
      prevtotsumD = maxd;
      b_n = D.size(0);
      cr = D.size(0);
      d.set_size(cr);
      for (i = 0; i < cr; i++) {
        d[i] = rtInf;
      }
      nidx.set_size(D.size(0));
      cr = D.size(0);
      for (i = 0; i < cr; i++) {
        nidx[i] = 1;
      }
      for (j = 0; j < 2; j++) {
        for (int b_i{0}; b_i < b_n; b_i++) {
          if (D[b_i + D.size(0) * j] < d[b_i]) {
            nidx[b_i] = static_cast<signed char>(j + 1);
            d[b_i] = D[b_i + D.size(0) * j];
          }
        }
      }
      cc = -1;
      for (int b_i{0}; b_i <= n; b_i++) {
        if ((nidx[b_i] != previdx[b_i]) &&
            (D[b_i + D.size(0) * (previdx[b_i] - 1)] > d[b_i])) {
          cc++;
          moved[cc] = b_i + 1;
          idx[b_i] = nidx[b_i];
        }
      }
      if (cc + 1 == 0) {
        *converged = true;
        exitg1 = 1;
      } else {
        b.set_size(moved.size(0));
        cr = moved.size(0);
        for (i = 0; i < cr; i++) {
          b[i] = false;
        }
        for (j = 0; j <= cc; j++) {
          b[idx[moved[j] - 1] - 1] = true;
          b[previdx[moved[j] - 1] - 1] = true;
        }
        nchanged = -1;
        i = b.size(0);
        for (j = 0; j < i; j++) {
          if (b[j]) {
            nchanged++;
            changed[nchanged] = j + 1;
          }
        }
      }
    }
  } while (exitg1 == 0);
}

static void gcentroids(double C[6], int counts[2],
                       const ::coder::array<double, 2U> &X,
                       const ::coder::array<int, 1U> &idx,
                       const int clusters[2], int nclusters)
{
  int clic;
  int n;
  n = X.size(0);
  for (int ic{0}; ic < nclusters; ic++) {
    clic = clusters[ic];
    counts[clic - 1] = 0;
    C[clic - 1] = rtNaN;
    C[clic + 1] = rtNaN;
    C[clic + 3] = rtNaN;
  }
  for (int ic{0}; ic < nclusters; ic++) {
    int cc;
    clic = clusters[ic] - 1;
    cc = 0;
    C[clic] = 0.0;
    C[clic + 2] = 0.0;
    C[clic + 4] = 0.0;
    for (int i{0}; i < n; i++) {
      if (idx[i] == clic + 1) {
        cc++;
        C[clic] += X[i];
        C[clic + 2] += X[i + X.size(0)];
        C[clic + 4] += X[i + X.size(0) * 2];
      }
    }
    counts[clusters[ic] - 1] = cc;
    C[clic] /= static_cast<double>(cc);
    C[clic + 2] /= static_cast<double>(cc);
    C[clic + 4] /= static_cast<double>(cc);
  }
}

void kmeans(::coder::array<double, 2U> &X, ::coder::array<int, 1U> &idxbest)
{
  array<double, 2U> D;
  array<double, 2U> b_X;
  array<double, 1U> d;
  array<double, 1U> sampleDist;
  array<int, 1U> idx;
  array<bool, 1U> wasnan;
  double Cbest[6];
  double b_index;
  double pt;
  int crows[2];
  int b_n;
  int c_n;
  int high_i;
  int low_i;
  int low_ip1;
  int mid_i;
  int n;
  bool DNeedsComputing;
  bool hadnans;
  n = X.size(0);
  wasnan.set_size(X.size(0));
  low_ip1 = X.size(0);
  for (mid_i = 0; mid_i < low_ip1; mid_i++) {
    wasnan[mid_i] = false;
  }
  hadnans = false;
  for (mid_i = 0; mid_i < n; mid_i++) {
    bool exitg1;
    low_ip1 = 0;
    exitg1 = false;
    while ((!exitg1) && (low_ip1 < 3)) {
      if (std::isnan(X[mid_i + X.size(0) * low_ip1])) {
        hadnans = true;
        wasnan[mid_i] = true;
        exitg1 = true;
      } else {
        low_ip1++;
      }
    }
  }
  if (hadnans) {
    high_i = wasnan.size(0) - 1;
    low_i = 0;
    for (mid_i = 0; mid_i <= high_i; mid_i++) {
      if (!wasnan[mid_i]) {
        low_i++;
      }
    }
    idx.set_size(low_i);
    low_i = 0;
    for (mid_i = 0; mid_i <= high_i; mid_i++) {
      if (!wasnan[mid_i]) {
        idx[low_i] = mid_i + 1;
        low_i++;
      }
    }
    b_X.set_size(idx.size(0), 3);
    low_ip1 = idx.size(0);
    for (mid_i = 0; mid_i < 3; mid_i++) {
      for (high_i = 0; high_i < low_ip1; high_i++) {
        b_X[high_i + b_X.size(0) * mid_i] =
            X[(idx[high_i] + X.size(0) * mid_i) - 1];
      }
    }
    X.set_size(b_X.size(0), 3);
    low_ip1 = b_X.size(0) * 3;
    for (mid_i = 0; mid_i < low_ip1; mid_i++) {
      X[mid_i] = b_X[mid_i];
    }
  }
  b_n = X.size(0) - 1;
  b_index = b_rand();
  for (mid_i = 0; mid_i < 6; mid_i++) {
    Cbest[mid_i] = 0.0;
  }
  mid_i = static_cast<int>(
      std::floor(b_index * static_cast<double>(X.size(0))) + 1.0);
  Cbest[0] = X[mid_i - 1];
  Cbest[2] = X[(mid_i + X.size(0)) - 1];
  Cbest[4] = X[(mid_i + X.size(0) * 2) - 1];
  D.set_size(X.size(0), 2);
  low_ip1 = X.size(0) << 1;
  for (mid_i = 0; mid_i < low_ip1; mid_i++) {
    D[mid_i] = 0.0;
  }
  c_n = X.size(0) - 1;
  for (high_i = 0; high_i <= c_n; high_i++) {
    b_index = X[high_i] - Cbest[0];
    D[high_i] = b_index * b_index;
  }
  for (low_ip1 = 0; low_ip1 < 2; low_ip1++) {
    for (high_i = 0; high_i <= c_n; high_i++) {
      b_index =
          X[high_i + X.size(0) * (low_ip1 + 1)] - Cbest[(low_ip1 + 1) << 1];
      D[high_i] = D[high_i] + b_index * b_index;
    }
  }
  low_ip1 = D.size(0);
  d.set_size(D.size(0));
  for (mid_i = 0; mid_i < low_ip1; mid_i++) {
    d[mid_i] = D[mid_i];
  }
  idx.set_size(X.size(0));
  low_ip1 = X.size(0);
  for (mid_i = 0; mid_i < low_ip1; mid_i++) {
    idx[mid_i] = 1;
  }
  sampleDist.set_size(X.size(0) + 1);
  low_ip1 = X.size(0);
  for (mid_i = 0; mid_i <= low_ip1; mid_i++) {
    sampleDist[mid_i] = 0.0;
  }
  DNeedsComputing = false;
  b_index = 0.0;
  sampleDist[0] = 0.0;
  for (mid_i = 0; mid_i <= b_n; mid_i++) {
    pt = D[mid_i];
    sampleDist[mid_i + 1] = sampleDist[mid_i] + pt;
    b_index += pt;
  }
  if ((b_index == 0.0) || (std::isinf(b_index) || std::isnan(b_index))) {
    double u;
    low_i = 1;
    idx.set_size(X.size(0));
    low_ip1 = X.size(0);
    for (mid_i = 0; mid_i < low_ip1; mid_i++) {
      idx[mid_i] = 0;
    }
    b_index = X.size(0);
    pt = 1.0 / static_cast<double>(X.size(0));
    u = b_rand();
    while (u > pt) {
      low_i++;
      b_index--;
      pt += (1.0 - pt) * (1.0 / b_index);
    }
    b_index = b_rand();
    b_index = std::floor(b_index);
    idx[0] = 0;
    idx[static_cast<int>(b_index)] = low_i;
    Cbest[1] = X[idx[0] - 1];
    Cbest[3] = X[(idx[0] + X.size(0)) - 1];
    Cbest[5] = X[(idx[0] + X.size(0) * 2) - 1];
    DNeedsComputing = true;
  } else {
    low_ip1 = sampleDist.size(0);
    for (mid_i = 0; mid_i < low_ip1; mid_i++) {
      sampleDist[mid_i] = sampleDist[mid_i] / b_index;
    }
    b_index = b_rand();
    high_i = sampleDist.size(0);
    low_i = 1;
    low_ip1 = 2;
    while (high_i > low_ip1) {
      mid_i = (low_i >> 1) + (high_i >> 1);
      if (((low_i & 1) == 1) && ((high_i & 1) == 1)) {
        mid_i++;
      }
      if (b_index >= sampleDist[mid_i - 1]) {
        low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }
    b_index = sampleDist[low_i - 1];
    if (b_index < 1.0) {
      while ((low_i <= b_n + 1) && (sampleDist[low_i] <= b_index)) {
        low_i++;
      }
    } else {
      while ((low_i >= 2) && (sampleDist[low_i - 2] >= b_index)) {
        low_i--;
      }
    }
    Cbest[1] = X[low_i - 1];
    Cbest[3] = X[(low_i + X.size(0)) - 1];
    Cbest[5] = X[(low_i + X.size(0) * 2) - 1];
    c_n = X.size(0) - 1;
    for (high_i = 0; high_i <= c_n; high_i++) {
      b_index = X[high_i] - Cbest[1];
      D[high_i + D.size(0)] = b_index * b_index;
    }
    for (low_ip1 = 0; low_ip1 < 2; low_ip1++) {
      for (high_i = 0; high_i <= c_n; high_i++) {
        b_index = X[high_i + X.size(0) * (low_ip1 + 1)] -
                  Cbest[((low_ip1 + 1) << 1) + 1];
        D[high_i + D.size(0)] = D[high_i + D.size(0)] + b_index * b_index;
      }
    }
    for (mid_i = 0; mid_i <= b_n; mid_i++) {
      pt = D[mid_i + D.size(0)];
      if (pt < d[mid_i]) {
        d[mid_i] = pt;
        idx[mid_i] = 2;
      }
    }
  }
  if (DNeedsComputing) {
    c_n = X.size(0) - 1;
    for (low_i = 0; low_i < 2; low_i++) {
      for (high_i = 0; high_i <= c_n; high_i++) {
        b_index = X[high_i] - Cbest[low_i];
        D[high_i + D.size(0) * low_i] = b_index * b_index;
      }
      for (low_ip1 = 0; low_ip1 < 2; low_ip1++) {
        for (high_i = 0; high_i <= c_n; high_i++) {
          b_index = X[high_i + X.size(0) * (low_ip1 + 1)] -
                    Cbest[low_i + ((low_ip1 + 1) << 1)];
          D[high_i + D.size(0) * low_i] =
              D[high_i + D.size(0) * low_i] + b_index * b_index;
        }
      }
    }
    c_n = D.size(0);
    d.set_size(D.size(0));
    low_ip1 = D.size(0);
    for (mid_i = 0; mid_i < low_ip1; mid_i++) {
      d[mid_i] = rtInf;
    }
    idx.set_size(D.size(0));
    low_ip1 = D.size(0);
    for (mid_i = 0; mid_i < low_ip1; mid_i++) {
      idx[mid_i] = 1;
    }
    for (low_ip1 = 0; low_ip1 < 2; low_ip1++) {
      for (mid_i = 0; mid_i < c_n; mid_i++) {
        pt = D[mid_i + D.size(0) * low_ip1];
        if (pt < d[mid_i]) {
          idx[mid_i] = low_ip1 + 1;
          d[mid_i] = pt;
        }
      }
    }
  }
  crows[0] = 0;
  crows[1] = 0;
  for (mid_i = 0; mid_i <= b_n; mid_i++) {
    crows[idx[mid_i] - 1]++;
  }
  batchUpdate(X, idx, Cbest, D, crows, &DNeedsComputing, &low_i);
  if (hadnans) {
    idxbest.set_size(n);
    low_ip1 = -1;
    for (mid_i = 0; mid_i < n; mid_i++) {
      if (wasnan[mid_i]) {
        idxbest[mid_i] = 0;
      } else {
        low_ip1++;
        idxbest[mid_i] = idx[low_ip1];
      }
    }
  } else {
    idxbest.set_size(idx.size(0));
    low_ip1 = idx.size(0);
    for (mid_i = 0; mid_i < low_ip1; mid_i++) {
      idxbest[mid_i] = idx[mid_i];
    }
  }
}

} // namespace coder

// End of code generation (kmeans.cpp)
