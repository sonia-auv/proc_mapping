//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PointCloudBundler.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "PointCloudBundler.h"
#include "Preprocessing.h"
#include "Subscriber.h"
#include "proc_mapping_data.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_types.h"
#include "rosReadField.h"
#include "rosReadXYZ.h"
#include "rt_nonfinite.h"
#include "sonar2NED.h"
#include "coder_array.h"
#include <cmath>
#include <stdio.h>
#include <string.h>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void PointCloudBundler::persistentDataStore_init()
{
  newSonarMsg = false;
  bundleStarted = false;
  newClearBundleMsg = false;
}

//
// Verifiy if we just stop the record.
//
// Arguments    : void
// Return Type  : bool
//
bool PointCloudBundler::step()
{
  coder::array<sensor_msgs_PointFieldStruct_T, 1U> sonarMsg_Fields;
  coder::array<double, 2U> b_this;
  coder::array<double, 2U> r2;
  coder::array<double, 2U> xyzi;
  coder::array<double, 1U> b_x;
  coder::array<double, 1U> r3;
  coder::array<double, 1U> r4;
  coder::array<double, 1U> x;
  coder::array<float, 2U> r;
  coder::array<float, 2U> r1;
  coder::array<unsigned char, 1U> sonarMsg_Data;
  coder::array<bool, 1U> c_x;
  coder::array<bool, 1U> rowsToDelete;
  geometry_msgs_PoseWithCovarianceStruct_T unusedExpr;
  double quat[4];
  double b_xyzi[3];
  double pos[3];
  double tx[3];
  unsigned int sonarMsg_Height;
  unsigned int sonarMsg_PointStep;
  unsigned int sonarMsg_Width;
  bool guard1{false};
  bool out;
  //         %% Step function
  guard1 = false;
  if (mLastBundleState) {
    //  Initial variables
    //  GET
    out = bundleStarted;
    if (!out) {
      //  Record finished.
      mLastBundleState = false;
      out = false;
      printf("INFO : proc mapping : sonar : Record finished. \n");
      fflush(stdout);
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    //  Recording or waiting.
    //  Initial variables
    //  GET
    out = newSonarMsg;
    if (out) {
      //  Initial variables
      //  GET
      out = bundleStarted;
      if (out) {
        double qnrm;
        int i1;
        int i2;
        int k;
        int nx;
        mPoseSub->get_LatestMessage(&unusedExpr);
        mSonarSub->get_LatestMessage(&sonarMsg_Height, &sonarMsg_Width,
                                     sonarMsg_Fields, &sonarMsg_PointStep,
                                     sonarMsg_Data);
        mPoseSub->get_LatestMessage(&unusedExpr);
        //         %% Adding to the point cloud.
        //  scan = rosReadLidarScan(sonarMsg);
        //  Getting the sub pose.
        pos[0] = unusedExpr.Pose.Position.X;
        pos[1] = unusedExpr.Pose.Position.Y;
        pos[2] = unusedExpr.Pose.Position.Z;
        qnrm =
            ((unusedExpr.Pose.Orientation.W * unusedExpr.Pose.Orientation.W +
              unusedExpr.Pose.Orientation.X * unusedExpr.Pose.Orientation.X) +
             unusedExpr.Pose.Orientation.Y * unusedExpr.Pose.Orientation.Y) +
            unusedExpr.Pose.Orientation.Z * unusedExpr.Pose.Orientation.Z;
        quat[0] = unusedExpr.Pose.Orientation.W / qnrm;
        quat[1] = -unusedExpr.Pose.Orientation.X / qnrm;
        quat[2] = -unusedExpr.Pose.Orientation.Y / qnrm;
        quat[3] = -unusedExpr.Pose.Orientation.Z / qnrm;
        xyzi.set_size(static_cast<int>(sonarMsg_Width), 4);
        nx = static_cast<int>(sonarMsg_Width) << 2;
        for (i2 = 0; i2 < nx; i2++) {
          xyzi[i2] = 0.0;
        }
        coder::rosReadXYZ(sonarMsg_Height, sonarMsg_Width, sonarMsg_Fields,
                          sonarMsg_PointStep, sonarMsg_Data, r);
        nx = r.size(0);
        for (i2 = 0; i2 < 3; i2++) {
          for (i1 = 0; i1 < nx; i1++) {
            xyzi[i1 + xyzi.size(0) * i2] = r[i1 + r.size(0) * i2];
          }
        }
        coder::rosReadField(sonarMsg_Height, sonarMsg_Width, sonarMsg_Fields,
                            sonarMsg_PointStep, sonarMsg_Data, r1);
        r2.set_size(r1.size(0), r1.size(1));
        nx = r1.size(0) * r1.size(1);
        for (i2 = 0; i2 < nx; i2++) {
          r2[i2] = r1[i2];
        }
        nx = xyzi.size(0);
        for (i2 = 0; i2 < nx; i2++) {
          xyzi[i2 + xyzi.size(0) * 3] = r2[i2];
        }
        nx = xyzi.size(0);
        x.set_size(xyzi.size(0));
        for (i2 = 0; i2 < nx; i2++) {
          qnrm = xyzi[i2];
          x[i2] = qnrm * qnrm;
        }
        nx = xyzi.size(0);
        r3.set_size(xyzi.size(0));
        for (i2 = 0; i2 < nx; i2++) {
          qnrm = xyzi[i2 + xyzi.size(0)];
          r3[i2] = qnrm * qnrm;
        }
        nx = xyzi.size(0);
        r4.set_size(xyzi.size(0));
        for (i2 = 0; i2 < nx; i2++) {
          qnrm = xyzi[i2 + xyzi.size(0) * 2];
          r4[i2] = qnrm * qnrm;
        }
        if (x.size(0) == 1) {
          nx = r3.size(0);
        } else {
          nx = x.size(0);
        }
        if ((x.size(0) == r3.size(0)) && (nx == r4.size(0))) {
          nx = x.size(0);
          for (i2 = 0; i2 < nx; i2++) {
            x[i2] = (x[i2] + r3[i2]) + r4[i2];
          }
        } else {
          b_binary_expand_op(x, r3, r4);
        }
        nx = x.size(0);
        for (k = 0; k < nx; k++) {
          x[k] = std::sqrt(x[k]);
        }
        nx = xyzi.size(0);
        b_x.set_size(xyzi.size(0));
        for (i2 = 0; i2 < nx; i2++) {
          qnrm = xyzi[i2];
          b_x[i2] = qnrm * qnrm;
        }
        nx = xyzi.size(0);
        r3.set_size(xyzi.size(0));
        for (i2 = 0; i2 < nx; i2++) {
          qnrm = xyzi[i2 + xyzi.size(0)];
          r3[i2] = qnrm * qnrm;
        }
        nx = xyzi.size(0);
        r4.set_size(xyzi.size(0));
        for (i2 = 0; i2 < nx; i2++) {
          qnrm = xyzi[i2 + xyzi.size(0) * 2];
          r4[i2] = qnrm * qnrm;
        }
        if (b_x.size(0) == 1) {
          nx = r3.size(0);
        } else {
          nx = b_x.size(0);
        }
        if ((b_x.size(0) == r3.size(0)) && (nx == r4.size(0))) {
          nx = b_x.size(0);
          for (i2 = 0; i2 < nx; i2++) {
            b_x[i2] = (b_x[i2] + r3[i2]) + r4[i2];
          }
        } else {
          b_binary_expand_op(b_x, r3, r4);
        }
        nx = b_x.size(0);
        for (k = 0; k < nx; k++) {
          b_x[k] = std::sqrt(b_x[k]);
        }
        if (xyzi.size(0) == 1) {
          nx = x.size(0);
        } else {
          nx = xyzi.size(0);
        }
        if ((xyzi.size(0) == x.size(0)) && (nx == b_x.size(0))) {
          nx = xyzi.size(0);
          c_x.set_size(xyzi.size(0));
          for (i2 = 0; i2 < nx; i2++) {
            qnrm = xyzi[i2 + xyzi.size(0) * 3];
            c_x[i2] = ((qnrm < mPreprocessing.param.minIntensity) ||
                       (qnrm > mPreprocessing.param.maxIntensity) ||
                       (x[i2] < mPreprocessing.param.minRange) ||
                       (b_x[i2] > mPreprocessing.param.maxRange));
          }
        } else {
          binary_expand_op(c_x, xyzi, this, x, b_x);
        }
        rowsToDelete.set_size(c_x.size(0));
        nx = c_x.size(0);
        for (i2 = 0; i2 < nx; i2++) {
          rowsToDelete[i2] = false;
        }
        nx = c_x.size(0);
        i1 = 0;
        i2 = 0;
        for (int j{0}; j < nx; j++) {
          bool exitg1;
          i1++;
          i2++;
          k = i1;
          exitg1 = false;
          while ((!exitg1) && ((nx > 0) && (k <= i2))) {
            if (c_x[k - 1]) {
              rowsToDelete[j] = true;
              exitg1 = true;
            } else {
              k += nx;
            }
          }
        }
        i1 = xyzi.size(0);
        nx = 0;
        i2 = rowsToDelete.size(0);
        for (k = 0; k < i2; k++) {
          nx += rowsToDelete[k];
        }
        nx = xyzi.size(0) - nx;
        i2 = 0;
        for (k = 0; k < i1; k++) {
          if ((k + 1 > rowsToDelete.size(0)) || (!rowsToDelete[k])) {
            xyzi[i2] = xyzi[k];
            xyzi[i2 + xyzi.size(0)] = xyzi[k + xyzi.size(0)];
            xyzi[i2 + xyzi.size(0) * 2] = xyzi[k + xyzi.size(0) * 2];
            xyzi[i2 + xyzi.size(0) * 3] = xyzi[k + xyzi.size(0) * 3];
            i2++;
          }
        }
        if (nx < 1) {
          nx = 0;
        }
        for (i2 = 0; i2 < 4; i2++) {
          for (i1 = 0; i1 < nx; i1++) {
            xyzi[i1 + nx * i2] = xyzi[i1 + xyzi.size(0) * i2];
          }
        }
        xyzi.set_size(nx, 4);
        if (nx - 1 >= 0) {
          tx[0] = mParam.parameters.sonar.translation.x;
          tx[1] = mParam.parameters.sonar.translation.y;
          tx[2] = mParam.parameters.sonar.translation.z;
          b_xyzi[2] = 0.0;
        }
        for (i2 = 0; i2 < nx; i2++) {
          double dv[4];
          b_xyzi[0] = xyzi[i2];
          b_xyzi[1] = xyzi[i2 + xyzi.size(0)];
          sonar2NED(pos, quat, tx, b_xyzi, dv);
          xyzi[i2] = dv[0];
          xyzi[i2 + xyzi.size(0)] = dv[1];
          xyzi[i2 + xyzi.size(0) * 2] = dv[2];
        }
        b_this.set_size(mBundle.size(0) + xyzi.size(0), 4);
        nx = mBundle.size(0);
        for (i2 = 0; i2 < 4; i2++) {
          for (i1 = 0; i1 < nx; i1++) {
            b_this[i1 + b_this.size(0) * i2] =
                mBundle[i1 + mBundle.size(0) * i2];
          }
        }
        nx = xyzi.size(0);
        for (i2 = 0; i2 < 4; i2++) {
          for (i1 = 0; i1 < nx; i1++) {
            b_this[(i1 + mBundle.size(0)) + b_this.size(0) * i2] =
                xyzi[i1 + xyzi.size(0) * i2];
          }
        }
        mBundle.set_size(b_this.size(0), 4);
        nx = b_this.size(0) * 4;
        for (i2 = 0; i2 < nx; i2++) {
          mBundle[i2] = b_this[i2];
        }
        //  Initial variables
        //  SET
        newSonarMsg = false;
      }
    }
    //  Clear the buffer if requested.
    //  Initial variables
    //  GET
    out = newClearBundleMsg;
    if (out) {
      printf("INFO : proc mapping : sonar : Clearing the sonar bundle \n");
      fflush(stdout);
      mBundle.set_size(1, 4);
      mBundle[0] = 0.0;
      mBundle[1] = 0.0;
      mBundle[2] = 0.0;
      mBundle[3] = 0.0;
      //  Initial variables
      //  SET
      newClearBundleMsg = false;
    }
    //  Initial variables
    //  GET
    mLastBundleState = bundleStarted;
    out = true;
  }
  return out;
}

//
// File trailer for PointCloudBundler.cpp
//
// [EOF]
//
