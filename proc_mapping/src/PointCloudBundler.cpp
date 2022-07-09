//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PointCloudBundler.cpp
//
// Code generation for function 'PointCloudBundler'
//

// Include files
#include "PointCloudBundler.h"
#include "Preprocessing.h"
#include "Subscriber.h"
#include "proc_mapping_data.h"
#include "proc_mapping_types.h"
#include "rosReadField.h"
#include "rosReadXYZ.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <stdio.h>
#include <string.h>

// Function Definitions
void PointCloudBundler::persistentDataStore_init()
{
  newSonarMsg = false;
  bundleStarted = false;
  newClearBundleMsg = false;
}

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
  double t0_Pose_Pose_Position_X;
  double t0_Pose_Pose_Position_Y;
  double t0_Pose_Pose_Position_Z;
  double t10;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t23;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  unsigned int sonarMsg_Height;
  unsigned int sonarMsg_PointStep;
  unsigned int sonarMsg_Width;
  bool out;
  //         %% Step function
  //  Verifiy if we just stop the record.
  if (mLastBundleState && (!bundleStarted)) {
    //  Initial variables
    //  GET
    //  Record finished.
    mLastBundleState = false;
    out = false;
  } else {
    //  Recording or waiting.
    //  Initial variables
    //  GET
    out = newSonarMsg;
    if (out && bundleStarted) {
      double qnrm;
      double quat_idx_0;
      double quat_idx_1;
      double quat_idx_2;
      int i1;
      int i2;
      int k;
      int nx;
      //  Initial variables
      //  GET
      mSonarSub->get_LatestMessage(&sonarMsg_Height, &sonarMsg_Width,
                                   sonarMsg_Fields, &sonarMsg_PointStep,
                                   sonarMsg_Data);
      mPoseSub->get_LatestMessage(
          &t0_Pose_Pose_Position_X, &t0_Pose_Pose_Position_Y,
          &t0_Pose_Pose_Position_Z, &t5, &t6, &t9, &t10);
      //         %% Adding to the point cloud.
      printf("INFO : proc mapping : Append to point cloud. \n");
      fflush(stdout);
      //  scan = rosReadLidarScan(sonarMsg);
      //  Getting the sub pose.
      printf("INFO : proc mapping : Pose received. \n");
      fflush(stdout);
      // fix
      qnrm = ((t10 * t10 + t5 * t5) + t6 * t6) + t9 * t9;
      quat_idx_0 = t10 / qnrm;
      quat_idx_1 = -t5 / qnrm;
      quat_idx_2 = -t6 / qnrm;
      qnrm = -t9 / qnrm;
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
        t5 = xyzi[i2];
        x[i2] = t5 * t5;
      }
      nx = xyzi.size(0);
      r3.set_size(xyzi.size(0));
      for (i2 = 0; i2 < nx; i2++) {
        t5 = xyzi[i2 + xyzi.size(0)];
        r3[i2] = t5 * t5;
      }
      nx = xyzi.size(0);
      r4.set_size(xyzi.size(0));
      for (i2 = 0; i2 < nx; i2++) {
        t5 = xyzi[i2 + xyzi.size(0) * 2];
        r4[i2] = t5 * t5;
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
        binary_expand_op(x, r3, r4);
      }
      nx = x.size(0);
      for (k = 0; k < nx; k++) {
        x[k] = std::sqrt(x[k]);
      }
      nx = xyzi.size(0);
      b_x.set_size(xyzi.size(0));
      for (i2 = 0; i2 < nx; i2++) {
        t5 = xyzi[i2];
        b_x[i2] = t5 * t5;
      }
      nx = xyzi.size(0);
      r3.set_size(xyzi.size(0));
      for (i2 = 0; i2 < nx; i2++) {
        t5 = xyzi[i2 + xyzi.size(0)];
        r3[i2] = t5 * t5;
      }
      nx = xyzi.size(0);
      r4.set_size(xyzi.size(0));
      for (i2 = 0; i2 < nx; i2++) {
        t5 = xyzi[i2 + xyzi.size(0) * 2];
        r4[i2] = t5 * t5;
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
        binary_expand_op(b_x, r3, r4);
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
          t10 = xyzi[i2 + xyzi.size(0) * 3];
          c_x[i2] = ((t10 < mPreprocessing.minIntensity) ||
                     (t10 > mPreprocessing.maxIntensity) ||
                     (x[i2] < mPreprocessing.minRange) ||
                     (b_x[i2] > mPreprocessing.maxRange));
        }
      } else {
        b_binary_expand_op(c_x, xyzi, this, x, b_x);
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
        t5 = quat_idx_1 * quat_idx_2 * 2.0;
        t6 = quat_idx_1 * qnrm * 2.0;
        t7 = quat_idx_2 * qnrm * 2.0;
        t8 = quat_idx_0 * quat_idx_1 * 2.0;
        t9 = quat_idx_0 * quat_idx_2 * 2.0;
        t10 = quat_idx_0 * qnrm * 2.0;
        quat_idx_0 = quat_idx_1 * quat_idx_1 * 2.0;
        quat_idx_1 = quat_idx_2 * quat_idx_2 * 2.0;
        qnrm = qnrm * qnrm * 2.0;
        t17 = t5 + t10;
        t18 = t6 + t9;
        t19 = t7 + t8;
        t20 = t5 + -t10;
        t21 = t6 + -t9;
        t22 = t7 + -t8;
        t23 = (quat_idx_0 + quat_idx_1) - 1.0;
        t8 = (quat_idx_0 + qnrm) - 1.0;
        t7 = (quat_idx_1 + qnrm) - 1.0;
      }
      for (i2 = 0; i2 < nx; i2++) {
        t10 = xyzi[i2];
        t5 = xyzi[i2 + xyzi.size(0)];
        // sonar2NED
        //     OUT1 = sonar2NED(IN1,IN2,IN3,IN4)
        //     This function was generated by the Symbolic Math Toolbox
        //     version 9.1. 11-May-2022 11:36:02
        xyzi[i2] = (((((t0_Pose_Pose_Position_X - 0.358 * t7) + 0.0 * t20) +
                      -0.118 * t18) -
                     t7 * t10) +
                    t20 * t5) +
                   t18 * 0.0;
        xyzi[i2 + xyzi.size(0)] =
            (((((t0_Pose_Pose_Position_Y + 0.358 * t17) - 0.0 * t8) +
               -0.118 * t22) +
              t17 * t10) -
             t8 * t5) +
            t22 * 0.0;
        xyzi[i2 + xyzi.size(0) * 2] =
            (((((t0_Pose_Pose_Position_Z + 0.358 * t21) + 0.0 * t19) -
               -0.118 * t23) +
              t21 * t10) +
             t19 * t5) -
            t23 * 0.0;
      }
      b_this.set_size(mBundle.size(0) + xyzi.size(0), 4);
      nx = mBundle.size(0);
      for (i2 = 0; i2 < 4; i2++) {
        for (i1 = 0; i1 < nx; i1++) {
          b_this[i1 + b_this.size(0) * i2] = mBundle[i1 + mBundle.size(0) * i2];
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
    //  Clear the buffer if requested.
    //  Initial variables
    //  GET
    out = newClearBundleMsg;
    if (out) {
      printf("INFO : proc mapping : Clearing the bundle \n");
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

// End of code generation (PointCloudBundler.cpp)
