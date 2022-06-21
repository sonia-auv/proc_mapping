//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping.cpp
//
// Code generation for function 'proc_mapping'
//

// Include files
#include "proc_mapping.h"
#include "Kdtree.h"
#include "PointCloudBundler.h"
#include "Publisher.h"
#include "Rate.h"
#include "Subscriber.h"
#include "any1.h"
#include "norm.h"
#include "pcdenoise.h"
#include "pcdownsample.h"
#include "pcfitplane.h"
#include "pcmerge.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "proc_mapping_data.h"
#include "proc_mapping_initialize.h"
#include "proc_mapping_types.h"
#include "rosReadField.h"
#include "rosReadXYZ.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_PointCloud2Struct.h"
#include "sensor_msgs_PointFieldStruct.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_posix_time.h"
#include "mlroscpp_pub.h"
#include "mlroscpp_rate.h"
#include <cstddef>
#include <cstring>
#include <stdio.h>
#include <string.h>

// Function Definitions
void proc_mapping()
{
  PointCloudBundler ptBundler;
  coder::b_pointCloud *b_this;
  coder::c_pointCloud b_lobj_1;
  coder::planeModel model1;
  coder::planeModel model2;
  coder::planeModel *b_model1;
  coder::planeModel *b_model2;
  coder::pointCloud lobj_3[5];
  coder::pointCloud filt;
  coder::pointCloud plane1;
  coder::pointCloud plane2;
  coder::pointCloud rawPT;
  coder::pointCloud remainCloud;
  coder::pointCloud *iobj_1;
  coder::pointCloud *output;
  coder::ros::Publisher lobj_1;
  coder::ros::Publisher *pub;
  coder::ros::Rate r;
  coder::ros::Subscriber *sub;
  coder::ros::b_Subscriber *b_sub;
  coder::ros::c_Subscriber *c_sub;
  coder::vision::internal::codegen::Kdtree b_lobj_2[5];
  coder::vision::internal::codegen::Kdtree lobj_5[3];
  coder::vision::internal::codegen::Kdtree lobj_2;
  coder::vision::internal::codegen::Kdtree lobj_4;
  coder::vision::internal::codegen::Kdtree *iobj_0;
  coder::array<double, 2U> b_ptBundler;
  coder::array<double, 2U> c_xyzi;
  coder::array<double, 2U> r2;
  coder::array<double, 2U> xyzi;
  coder::array<double, 1U> b_xyzi;
  coder::array<double, 1U> v;
  coder::array<float, 2U> XYZ;
  coder::array<float, 2U> r1;
  coder::array<float, 1U> RGB;
  coder::array<bool, 1U> d_xyzi;
  coder::array<bool, 1U> idx;
  sensor_msgs_PointCloud2Struct_T pack;
  double poseMsg_Orientation_Z;
  double poseMsg_Position_X;
  double poseMsg_Position_Y;
  double poseMsg_Position_Z;
  double t10;
  double t12;
  double t13;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  int i;
  if (!isInitialized_proc_mapping) {
    proc_mapping_initialize();
  }
  //  Variables
  r.init();
  //  Proc_mapping startup
  //         %% ROS Node constructor
  pub = lobj_1.init();
  ptBundler.bigCloud.matlabCodegenIsDeleted = true;
  ptBundler.matlabCodegenIsDeleted = true;
  for (i = 0; i < 5; i++) {
    lobj_3[i].matlabCodegenIsDeleted = true;
  }
  filt.matlabCodegenIsDeleted = true;
  //         %% ROS Spin
  MATLABRate_reset(r.RateHelper);
  coder::tic(&t10, &t5);
  printf("INFO : proc mapping : Node is started. \n");
  fflush(stdout);
  printf("INFO : proc mapping : Wait for point cloud. \n");
  fflush(stdout);
  //  Instances
  //         %% PointCloudBundler Constructor
  b_this = &ptBundler.bigCloud;
  b_this->Location[0] = 0.0;
  b_this->Location[1] = 0.0;
  b_this->Location[2] = 0.0;
  b_this->Color.set_size(0, 0);
  b_this->Normal.set_size(0, 0);
  b_this->Intensity.set_size(1, 1);
  b_this->Intensity[0] = 0.0;
  b_this->b_Kdtree = &b_this->_pobj0;
  b_this->matlabCodegenIsDeleted = false;
  ptBundler.bundle.set_size(3, 4);
  for (i = 0; i < 12; i++) {
    ptBundler.bundle[i] = 0.0;
  }
  ptBundler.bundle.set_size(1, 4);
  ptBundler.bundle[0] = 0.0;
  ptBundler.bundle[1] = 0.0;
  ptBundler.bundle[2] = 0.0;
  ptBundler.bundle[3] = 0.0;
  //  Subscribers
  sub = ptBundler._pobj2.init();
  ptBundler.startStopSub = sub;
  b_sub = ptBundler._pobj1.init();
  ptBundler.poseSub = b_sub;
  c_sub = ptBundler._pobj0.init();
  ptBundler.sonarSub = c_sub;
  ptBundler.lastBundleState = false;
  ptBundler.matlabCodegenIsDeleted = false;
  while (1) {
    //         %% Step function
    if (ptBundler.lastBundleState && (!bundleStarted)) {
      int nrows;
      unsigned int q0;
      //  Initial variables
      //  GET
      //  Record finished.
      ptBundler.lastBundleState = false;
      printf("INFO : proc mapping : Not bundling. \n");
      fflush(stdout);
      filt.matlabCodegenDestructor();
      //         %% Getters / Setters
      xyzi.set_size(ptBundler.bundle.size(0), 4);
      nrows = ptBundler.bundle.size(0) * 4;
      for (i = 0; i < nrows; i++) {
        xyzi[i] = ptBundler.bundle[i];
      }
      b_lobj_1.matlabCodegenIsDeleted = true;
      //  Create the point cloud and apply denoise filter plus a downsample.
      nrows = xyzi.size(0);
      rawPT.Location.set_size(xyzi.size(0), 3);
      for (i = 0; i < 3; i++) {
        for (int nrowx{0}; nrowx < nrows; nrowx++) {
          rawPT.Location[nrowx + rawPT.Location.size(0) * i] =
              xyzi[nrowx + xyzi.size(0) * i];
        }
      }
      rawPT.Color.set_size(0, 0);
      rawPT.Normal.set_size(0, 0);
      nrows = xyzi.size(0);
      rawPT.Intensity.set_size(xyzi.size(0));
      for (i = 0; i < nrows; i++) {
        rawPT.Intensity[i] = xyzi[i + xyzi.size(0) * 3];
      }
      coder::pointclouds::internal::codegen::pc::pointCloudArray b_r;
      rawPT.RangeData.set_size(0, 0);
      rawPT.PointCloudArrayData.set_size(1, 1);
      rawPT.PointCloudArrayData[0] = b_r;
      rawPT.Kdtree = &lobj_2;
      rawPT.matlabCodegenIsDeleted = false;
      coder::pcdenoise(coder::pcdownsample(&rawPT, &b_lobj_1), &lobj_4, &filt);
      b_lobj_1.matlabCodegenDestructor();
      rawPT.matlabCodegenDestructor();
      for (i = 0; i < 5; i++) {
        lobj_3[i].matlabCodegenDestructor();
      }
      iobj_0 = &b_lobj_2[0];
      iobj_1 = &lobj_3[0];
      plane2.matlabCodegenIsDeleted = true;
      remainCloud.matlabCodegenIsDeleted = true;
      plane1.matlabCodegenIsDeleted = true;
      //  Get first wall
      coder::pcfitplane(&filt, &model1, &b_model1, v, b_xyzi);
      filt.b_select(v, &lobj_5[0], &plane1);
      filt.b_select(b_xyzi, &lobj_5[1], &remainCloud);
      coder::pcfitplane(&remainCloud, &model2, &b_model2, v, b_xyzi);
      remainCloud.b_select(v, &lobj_5[2], &plane2);
      output = coder::pcmerge(&plane1, &plane2, &iobj_0[0], &iobj_1[0]);
      plane1.matlabCodegenDestructor();
      remainCloud.matlabCodegenDestructor();
      plane2.matlabCodegenDestructor();
      XYZ.set_size(output->Location.size(0), 3);
      nrows = output->Location.size(0) * 3;
      for (i = 0; i < nrows; i++) {
        XYZ[i] = static_cast<float>(output->Location[i]);
      }
      RGB.set_size(output->Intensity.size(0));
      nrows = output->Intensity.size(0);
      for (i = 0; i < nrows; i++) {
        RGB[i] = static_cast<float>(output->Intensity[i]);
      }
      sensor_msgs_PointCloud2Struct(&pack);
      pack.Header.FrameId.set_size(1, 4);
      pack.Header.FrameId[0] = 'B';
      pack.Header.FrameId[1] = 'O';
      pack.Header.FrameId[2] = 'D';
      pack.Header.FrameId[3] = 'Y';
      //  Calculate number of points
      //  Assign metadata
      pack.Height = 1U;
      pack.Width = static_cast<unsigned int>(XYZ.size(0));
      pack.PointStep = 16U;
      t10 = 16.0 * static_cast<double>(XYZ.size(0));
      if (t10 < 4.294967296E+9) {
        q0 = static_cast<unsigned int>(t10);
      } else {
        q0 = MAX_uint32_T;
      }
      pack.RowStep = q0;
      //  Assign point field data
      pack.Data.set_size(static_cast<int>(q0));
      nrows = static_cast<int>(q0);
      for (i = 0; i < nrows; i++) {
        pack.Data[i] = 0U;
      }
      sensor_msgs_PointFieldStruct(&(pack.Fields.data())[0]);
      pack.Fields[0].Name[0] = 'x';
      pack.Fields[0].Datatype = 7U;
      pack.Fields[0].Count = 1U;
      sensor_msgs_PointFieldStruct(&pack.Fields[1]);
      pack.Fields[1].Name[0] = 'y';
      pack.Fields[1].Datatype = 7U;
      pack.Fields[1].Count = 1U;
      sensor_msgs_PointFieldStruct(&pack.Fields[2]);
      pack.Fields[2].Name[0] = 'z';
      pack.Fields[2].Datatype = 7U;
      pack.Fields[2].Count = 1U;
      sensor_msgs_PointFieldStruct(&pack.Fields[3]);
      for (i = 0; i < 9; i++) {
        pack.Fields[3].Name[i] = cv[i];
      }
      pack.Fields[3].Datatype = 7U;
      pack.Fields[3].Count = 1U;
      pack.Fields[0].Offset = 0U;
      pack.Fields[1].Offset = 4U;
      pack.Fields[2].Offset = 8U;
      pack.Fields[3].Offset = 16U;
      //  Assign raw point cloud data in uint8 format
      i = XYZ.size(0);
      for (nrows = 0; nrows < i; nrows++) {
        float x[3];
        unsigned int qY;
        unsigned char y[12];
        unsigned char b_y[4];
        t10 = ((static_cast<double>(nrows) + 1.0) - 1.0) * 16.0;
        if (t10 < 4.294967296E+9) {
          q0 = static_cast<unsigned int>(t10);
        } else {
          q0 = MAX_uint32_T;
        }
        qY = q0 + 1U;
        if (q0 + 1U < q0) {
          qY = MAX_uint32_T;
        }
        x[0] = XYZ[nrows];
        x[1] = XYZ[nrows + XYZ.size(0)];
        x[2] = XYZ[nrows + XYZ.size(0) * 2];
        std::memcpy((void *)&y[0], (void *)&x[0],
                    (unsigned int)((size_t)12 * sizeof(unsigned char)));
        for (int nrowx{0}; nrowx < 12; nrowx++) {
          q0 = qY + nrowx;
          if (q0 < qY) {
            q0 = MAX_uint32_T;
          }
          pack.Data[static_cast<int>(q0) - 1] = y[nrowx];
        }
        std::memcpy((void *)&b_y[0], (void *)&RGB[nrows],
                    (unsigned int)((size_t)4 * sizeof(unsigned char)));
        q0 = qY + 12U;
        if (qY + 12U < qY) {
          q0 = MAX_uint32_T;
        }
        pack.Data[static_cast<int>(q0) - 1] = b_y[0];
        q0 = qY + 13U;
        if (qY + 13U < qY) {
          q0 = MAX_uint32_T;
        }
        pack.Data[static_cast<int>(q0) - 1] = b_y[1];
        q0 = qY + 14U;
        if (qY + 14U < qY) {
          q0 = MAX_uint32_T;
        }
        pack.Data[static_cast<int>(q0) - 1] = b_y[2];
        q0 = qY + 15U;
        if (qY + 15U < qY) {
          q0 = MAX_uint32_T;
        }
        pack.Data[static_cast<int>(q0) - 1] = b_y[3];
        //  NOTE: The 16th byte remains empty
      }
      MATLABPUBLISHER_publish(pub->PublisherHelper, &pack);
    } else {
      bool out;
      //  Recording or waiting.
      //  Initial variables
      //  GET
      out = newSonarMsg;
      if (out && bundleStarted) {
        int nrows;
        int nrowx;
        //  Initial variables
        //  GET
        ptBundler.sonarSub->get_LatestMessage(&pack);
        ptBundler.poseSub->get_LatestMessage(
            &poseMsg_Position_X, &poseMsg_Position_Y, &poseMsg_Position_Z, &t13,
            &t12, &poseMsg_Orientation_Z, &t10);
        //         %% Adding to the point cloud.
        printf("INFO : proc mapping : Append to point cloud. \n");
        fflush(stdout);
        //  scan = rosReadLidarScan(sonarMsg);
        //  Getting the sub pose.
        printf("INFO : proc mapping : Pose received. \n");
        fflush(stdout);
        xyzi.set_size(static_cast<int>(pack.Width), 4);
        nrows = static_cast<int>(pack.Width) << 2;
        for (i = 0; i < nrows; i++) {
          xyzi[i] = 0.0;
        }
        coder::rosReadXYZ(pack.Height, pack.Width, pack.Fields, pack.PointStep,
                          pack.Data, XYZ);
        nrows = XYZ.size(0);
        for (i = 0; i < 3; i++) {
          for (nrowx = 0; nrowx < nrows; nrowx++) {
            xyzi[nrowx + xyzi.size(0) * i] = XYZ[nrowx + XYZ.size(0) * i];
          }
        }
        //  Temporary swap.
        nrows = xyzi.size(0);
        v.set_size(xyzi.size(0));
        for (i = 0; i < nrows; i++) {
          v[i] = xyzi[i];
        }
        nrows = xyzi.size(0) - 1;
        b_xyzi.set_size(xyzi.size(0));
        for (i = 0; i <= nrows; i++) {
          b_xyzi[i] = xyzi[i + xyzi.size(0)];
        }
        nrows = b_xyzi.size(0);
        for (i = 0; i < nrows; i++) {
          xyzi[i] = b_xyzi[i];
        }
        nrows = v.size(0);
        for (i = 0; i < nrows; i++) {
          xyzi[i + xyzi.size(0)] = v[i];
        }
        coder::rosReadField(pack.Height, pack.Width, pack.Fields,
                            pack.PointStep, pack.Data, r1);
        r2.set_size(r1.size(0), r1.size(1));
        nrows = r1.size(0) * r1.size(1);
        for (i = 0; i < nrows; i++) {
          r2[i] = r1[i];
        }
        nrows = xyzi.size(0);
        for (i = 0; i < nrows; i++) {
          xyzi[i + xyzi.size(0) * 3] = r2[i];
        }
        nrows = xyzi.size(0);
        c_xyzi.set_size(xyzi.size(0), 3);
        for (i = 0; i < 3; i++) {
          for (nrowx = 0; nrowx < nrows; nrowx++) {
            c_xyzi[nrowx + c_xyzi.size(0) * i] = xyzi[nrowx + xyzi.size(0) * i];
          }
        }
        out = (coder::b_norm(c_xyzi) > 5.0);
        nrows = xyzi.size(0);
        d_xyzi.set_size(xyzi.size(0));
        for (i = 0; i < nrows; i++) {
          d_xyzi[i] = ((xyzi[i + xyzi.size(0) * 3] < 0.07) && out);
        }
        coder::any(d_xyzi, idx);
        nrowx = xyzi.size(0);
        nrows = 0;
        i = idx.size(0);
        for (int k{0}; k < i; k++) {
          nrows += idx[k];
        }
        nrows = xyzi.size(0) - nrows;
        i = 0;
        for (int k{0}; k < nrowx; k++) {
          if ((k + 1 > idx.size(0)) || (!idx[k])) {
            xyzi[i] = xyzi[k];
            xyzi[i + xyzi.size(0)] = xyzi[k + xyzi.size(0)];
            xyzi[i + xyzi.size(0) * 2] = xyzi[k + xyzi.size(0) * 2];
            xyzi[i + xyzi.size(0) * 3] = xyzi[k + xyzi.size(0) * 3];
            i++;
          }
        }
        if (nrows < 1) {
          nrows = 0;
        }
        for (i = 0; i < 4; i++) {
          for (nrowx = 0; nrowx < nrows; nrowx++) {
            xyzi[nrowx + nrows * i] = xyzi[nrowx + xyzi.size(0) * i];
          }
        }
        xyzi.set_size(nrows, 4);
        // xyzPoints = zeros([size(xyzi, 1), 3]);
        if (nrows - 1 >= 0) {
          double t11;
          t5 = t13 * t12 * 2.0;
          t6 = t13 * poseMsg_Orientation_Z * 2.0;
          t7 = t12 * poseMsg_Orientation_Z * 2.0;
          t8 = t10 * t13 * 2.0;
          t9 = t10 * t12 * 2.0;
          t10 = t10 * poseMsg_Orientation_Z * 2.0;
          t11 = t13 * t13 * 2.0;
          t12 = t12 * t12 * 2.0;
          t13 = poseMsg_Orientation_Z * poseMsg_Orientation_Z * 2.0;
          t17 = t5 + t10;
          t18 = t6 + t9;
          t19 = t7 + t8;
          t20 = t5 + -t10;
          t21 = t6 + -t9;
          t7 += -t8;
          t8 = (t11 + t12) - 1.0;
          t9 = (t11 + t13) - 1.0;
          t6 = (t12 + t13) - 1.0;
        }
        for (i = 0; i < nrows; i++) {
          t10 = xyzi[i];
          t5 = xyzi[i + xyzi.size(0)];
          // sonar2NED
          //     OUT1 = sonar2NED(IN1,IN2,IN3,IN4)
          //     This function was generated by the Symbolic Math Toolbox
          //     version 9.1. 11-May-2022 11:36:02
          xyzi[i] = (((((poseMsg_Position_X - 0.358 * t6) + 0.0 * t20) +
                       -0.118 * t18) -
                      t6 * t10) +
                     t20 * t5) +
                    t18 * 0.0;
          xyzi[i + xyzi.size(0)] =
              (((((poseMsg_Position_Y + 0.358 * t17) - 0.0 * t9) +
                 -0.118 * t7) +
                t17 * t10) -
               t9 * t5) +
              t7 * 0.0;
          xyzi[i + xyzi.size(0) * 2] =
              (((((poseMsg_Position_Z + 0.358 * t21) + 0.0 * t19) -
                 -0.118 * t8) +
                t21 * t10) +
               t19 * t5) -
              t8 * 0.0;
        }
        b_ptBundler.set_size(ptBundler.bundle.size(0) + xyzi.size(0), 4);
        for (i = 0; i < 4; i++) {
          nrows = ptBundler.bundle.size(0);
          for (nrowx = 0; nrowx < nrows; nrowx++) {
            b_ptBundler[nrowx + b_ptBundler.size(0) * i] =
                ptBundler.bundle[nrowx + ptBundler.bundle.size(0) * i];
          }
        }
        nrows = xyzi.size(0);
        for (i = 0; i < 4; i++) {
          for (nrowx = 0; nrowx < nrows; nrowx++) {
            b_ptBundler[(nrowx + ptBundler.bundle.size(0)) +
                        b_ptBundler.size(0) * i] =
                xyzi[nrowx + xyzi.size(0) * i];
          }
        }
        ptBundler.bundle.set_size(b_ptBundler.size(0), 4);
        nrows = b_ptBundler.size(0) * 4;
        for (i = 0; i < nrows; i++) {
          ptBundler.bundle[i] = b_ptBundler[i];
        }
        //  Initial variables
        //  SET
        newSonarMsg = false;
      }
      //  Initial variables
      //  GET
      ptBundler.lastBundleState = bundleStarted;
      //  fprintf('INFO : proc mapping : Bundling or waiting. \n');
    }
    MATLABRate_sleep(r.RateHelper);
    coder::toc(r.PreviousPeriod.tv_sec, r.PreviousPeriod.tv_nsec);
    coder::tic(&r.PreviousPeriod.tv_sec, &r.PreviousPeriod.tv_nsec);
  }
}

// End of code generation (proc_mapping.cpp)
