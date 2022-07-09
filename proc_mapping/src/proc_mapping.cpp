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
#include "Buoys.h"
#include "Kdtree.h"
#include "ParameterTree.h"
#include "PointCloudBundler.h"
#include "Preprocessing.h"
#include "Publisher.h"
#include "Rate.h"
#include "RosNode.h"
#include "Subscriber.h"
#include "affine3d.h"
#include "combineVectorElements.h"
#include "find.h"
#include "kmeans.h"
#include "pcdenoise.h"
#include "pcdownsample.h"
#include "pcfitplane.h"
#include "pcregistericp.h"
#include "pcsegdist.h"
#include "pctransform.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "proc_mapping_data.h"
#include "proc_mapping_initialize.h"
#include "proc_mapping_internal_types.h"
#include "quat2rotm.h"
#include "quatUtilities.h"
#include "rigid3d.h"
#include "rigid3dImpl.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_posix_time.h"
#include "mlroscpp_param.h"
#include "mlroscpp_rate.h"
#include <algorithm>
#include <functional>
#include <stdio.h>
#include <string.h>

// Function Definitions
void proc_mapping()
{
  static coder::b_pointCloud b_lobj_2;
  static coder::b_pointCloud buoyTformed;
  Buoys buoys;
  PointCloudBundler b_lobj_1;
  RosNode rosNode;
  coder::c_pointCloud d_lobj_1;
  coder::images::internal::rigid3dImpl r1;
  coder::planeModel c_model;
  coder::planeModel model;
  coder::planeModel *b_model;
  coder::planeModel *d_model;
  coder::pointCloud b_clusterPT;
  coder::pointCloud b_plane;
  coder::pointCloud clusterPT;
  coder::pointCloud d_lobj_2;
  coder::pointCloud filt;
  coder::pointCloud pct;
  coder::pointCloud plane;
  coder::pointCloud rawPT;
  coder::pointCloud *ptCloudOut;
  coder::pointCloud *subPT;
  coder::rigid3d tf;
  coder::rigid3d tformICP;
  coder::ros::ParameterTree c_lobj_1;
  coder::ros::ParameterTree lobj_1;
  coder::ros::Publisher lobj_3;
  coder::ros::Rate r;
  coder::ros::Subscriber *sub;
  coder::ros::b_Publisher lobj_2;
  coder::ros::b_Subscriber *b_sub;
  coder::ros::c_Subscriber *c_sub;
  coder::ros::d_Subscriber *d_sub;
  coder::ros::e_Subscriber *e_sub;
  coder::vision::internal::codegen::Kdtree c_lobj_3[2];
  coder::vision::internal::codegen::Kdtree b_lobj_3;
  coder::vision::internal::codegen::Kdtree c_lobj_2;
  coder::vision::internal::codegen::Kdtree d_lobj_3;
  coder::vision::internal::codegen::Kdtree e_lobj_1;
  coder::vision::internal::codegen::Kdtree e_lobj_3;
  coder::vision::internal::codegen::Kdtree f_lobj_1;
  coder::vision::internal::codegen::Kdtree g_lobj_1;
  coder::vision::internal::codegen::Kdtree lobj_4;
  coder::vision::internal::codegen::Kdtree *iobj_0;
  coder::array<double, 2U> b_bundle;
  coder::array<double, 2U> b_pct;
  coder::array<double, 2U> bundle;
  coder::array<double, 2U> c_pct;
  coder::array<double, 2U> c_r;
  coder::array<double, 2U> goodCluster;
  coder::array<double, 2U> meanError;
  coder::array<double, 2U> r5;
  coder::array<double, 2U> r6;
  coder::array<double, 1U> a__1;
  coder::array<double, 1U> c_bundle;
  coder::array<int, 2U> r3;
  coder::array<unsigned int, 1U> b_r;
  coder::array<int, 1U> kmeansIndex;
  coder::array<int, 1U> r4;
  coder::array<unsigned char, 2U> c;
  coder::array<char, 2U> in;
  coder::array<char, 2U> parameterName;
  coder::array<bool, 2U> b_goodCluster;
  coder::array<bool, 2U> r2;
  coder::array<bool, 1U> b_kmeansIndex;
  double b_varargin_1[16];
  double area;
  double expl_temp;
  double param_maxRange;
  double param_minRange;
  int b_loop_ub;
  int i;
  int loop_ub;
  int unnamed_idx_0;
  bool nameExists;
  if (!isInitialized_proc_mapping) {
    proc_mapping_initialize();
  }
  //  Variables
  r.init();
  //  Proc_mapping startup
  //         %% ROS Node constructor
  //  ROS Publishers
  lobj_3.init();
  lobj_2.init();
  //  ROS parameters
  lobj_1.ParameterHelper = MATLABROSParameter();
  UNUSED_PARAM(lobj_1.ParameterHelper);
  // ROSPARAM Construct an instance of this class
  //    Detailed explanation goes here
  //         %% get rosparam number
  coder::ros::ParameterTree::canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (i = 0; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    coder::ros::ParameterTree::canonicalizeName(&lobj_1, in);
    parameterName.set_size(1, in.size(1) + 1);
    loop_ub = in.size(1);
    for (i = 0; i < loop_ub; i++) {
      parameterName[i] = in[i];
    }
    parameterName[in.size(1)] = '\x00';
    rosNode.param.preprocessing.minIntensity = 0.0;
    std::mem_fn (&MATLABROSParameter::getParameter<double>)(
        &lobj_1.ParameterHelper, &parameterName[0],
        &rosNode.param.preprocessing.minIntensity);
    //  fprintf("%s : %f \n", value, val);
  } else {
    rosNode.param.preprocessing.minIntensity = 0.1;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::b_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (i = 0; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    rosNode.param.preprocessing.maxIntensity = lobj_1.get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    rosNode.param.preprocessing.maxIntensity = 1.0;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::c_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (i = 0; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    rosNode.param.preprocessing.minRange = lobj_1.b_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    rosNode.param.preprocessing.minRange = 0.1;
  }
  //         %% get rosparam number
  coder::ros::ParameterTree::d_canonicalizeName(&lobj_1, in);
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (i = 0; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
      lobj_1.ParameterHelper, &parameterName[0]);
  if (nameExists) {
    rosNode.param.preprocessing.maxRange = lobj_1.c_get();
    //  fprintf("%s : %f \n", value, val);
  } else {
    rosNode.param.preprocessing.maxRange = 5.0;
  }
  //  seconds
  rosNode.counter = 0.0;
  filt.matlabCodegenIsDeleted = true;
  b_lobj_2.matlabCodegenIsDeleted = true;
  //         %% ROS Spin
  MATLABRate_reset(r.RateHelper);
  coder::tic(&area, &expl_temp);
  printf("INFO : proc mapping : Node is started. \n");
  fflush(stdout);
  printf("INFO : proc mapping : Wait for point cloud. \n");
  fflush(stdout);
  //  Instances
  //         %% PointCloudBundler Constructor
  //  Graphics functions
  b_lobj_1.mBundle.set_size(3, 4);
  for (i = 0; i < 12; i++) {
    b_lobj_1.mBundle[i] = 0.0;
  }
  b_lobj_1.mBundle.set_size(1, 4);
  b_lobj_1.mBundle[0] = 0.0;
  b_lobj_1.mBundle[1] = 0.0;
  b_lobj_1.mBundle[2] = 0.0;
  b_lobj_1.mBundle[3] = 0.0;
  //  Subscribers
  sub = b_lobj_1._pobj4.init();
  b_lobj_1.mStartStopSub = sub;
  b_sub = b_lobj_1._pobj3.init();
  b_lobj_1.mClearBundleSub = b_sub;
  c_sub = b_lobj_1._pobj2.init();
  b_lobj_1.mPoseSub = c_sub;
  d_sub = b_lobj_1._pobj1.init();
  b_lobj_1.mSonarSub = d_sub;
  e_sub = b_lobj_1._pobj0.init();
  b_lobj_1.mImageSub = e_sub;
  b_lobj_1.mLastBundleState = false;
  //  ROS params
  b_lobj_1.param = rosNode.param;
  area = b_lobj_1.param.preprocessing.minIntensity;
  expl_temp = b_lobj_1.param.preprocessing.maxIntensity;
  param_minRange = b_lobj_1.param.preprocessing.minRange;
  param_maxRange = b_lobj_1.param.preprocessing.maxRange;
  //         %% Preprocessing Constructor
  //  ROS Subscribers
  b_lobj_1.mPreprocessing.minIntensity = area;
  b_lobj_1.mPreprocessing.maxIntensity = expl_temp;
  b_lobj_1.mPreprocessing.minRange = param_minRange;
  b_lobj_1.mPreprocessing.maxRange = param_maxRange;
  while (1) {
    if (!b_lobj_1.step()) {
      printf("INFO : proc mapping : Not bundling. \n");
      fflush(stdout);
      //         %% Getters / Setters
      bundle.set_size(b_lobj_1.mBundle.size(0), 4);
      loop_ub = b_lobj_1.mBundle.size(0) * 4;
      for (i = 0; i < loop_ub; i++) {
        bundle[i] = b_lobj_1.mBundle[i];
      }
      if (bundle.size(0) > 1) {
        double varargin_1[9];
        double q[4];
        double normal[3];
        int i1;
        int varargin_1_tmp;
        //  Create and filter pointcloud form bundle
        filt.matlabCodegenDestructor();
        rawPT.matlabCodegenIsDeleted = true;
        d_lobj_1.matlabCodegenIsDeleted = true;
        //  Create the point cloud and apply denoise filter plus a downsample.
        loop_ub = bundle.size(0);
        b_bundle.set_size(bundle.size(0), 3);
        for (i = 0; i < 3; i++) {
          for (i1 = 0; i1 < loop_ub; i1++) {
            b_bundle[i1 + b_bundle.size(0) * i] =
                bundle[i1 + bundle.size(0) * i];
          }
        }
        loop_ub = bundle.size(0);
        c_bundle.set_size(bundle.size(0));
        for (i = 0; i < loop_ub; i++) {
          c_bundle[i] = bundle[i + bundle.size(0) * 3];
        }
        rawPT.init(b_bundle, c_bundle, &c_lobj_2);
        coder::pcdenoise(coder::pcdownsample(&rawPT, &d_lobj_1), &b_lobj_3,
                         &filt);
        d_lobj_1.matlabCodegenDestructor();
        rawPT.matlabCodegenDestructor();
        b_lobj_2.matlabCodegenDestructor();
        // WALLCORNER Construct an instance of this class
        //    Detailed explanation goes here
        buoys.buoyPT = b_lobj_2.init(&e_lobj_1);
        buoys.buoyPT->Intensity.set_size(7439, 1);
        for (i = 0; i < 7439; i++) {
          buoys.buoyPT->Intensity[i] = 0.1F;
        }
        buoys.buoyPT->Normal.set_size(7439, 3);
        for (i = 0; i < 22317; i++) {
          buoys.buoyPT->Normal[i] = 0.0F;
        }
        d_lobj_2.matlabCodegenIsDeleted = true;
        clusterPT.matlabCodegenIsDeleted = true;
        //  Get clusters
        coder::pcsegdist(&filt, b_r, &area);
        i = static_cast<int>(area);
        goodCluster.set_size(1, i);
        if (static_cast<int>(area) - 1 >= 0) {
          unnamed_idx_0 = b_r.size(0);
          b_loop_ub = b_r.size(0);
        }
        for (int b_i{0}; b_i < i; b_i++) {
          plane.matlabCodegenIsDeleted = true;
          b_clusterPT.matlabCodegenIsDeleted = true;
          //  extract pointCloud
          r2.set_size(unnamed_idx_0, 1);
          for (i1 = 0; i1 < b_loop_ub; i1++) {
            r2[i1] = (b_r[i1] == b_i + 1U);
          }
          filt.b_select(r2, &c_lobj_3[0], &b_clusterPT);
          //  fit plane on cluster
          coder::pcfitplane(&b_clusterPT, &model, &b_model, c_bundle, a__1,
                            meanError);
          iobj_0 = &c_lobj_3[1];
          b_clusterPT.subsetImpl(c_bundle, b_bundle, c, meanError, a__1, c_r);
          ptCloudOut = plane.init(b_bundle, c, meanError, a__1, iobj_0);
          ptCloudOut->RangeData.set_size(c_r.size(0), c_r.size(1));
          loop_ub = c_r.size(0) * c_r.size(1);
          for (i1 = 0; i1 < loop_ub; i1++) {
            ptCloudOut->RangeData[i1] = c_r[i1];
          }
          double p[3];
          //  Get Z normal
          normal[2] = model.Parameters[2];
          //  Ratio in plane
          loop_ub = b_clusterPT.Location.size(0);
          //  Extract pose of the plane.
          quatUtilities::getOrientedPointOnPlanarFace(&model, &plane, p, q);
          //  extract bounding box
          pct.matlabCodegenIsDeleted = true;
          coder::quat2rotm(q, varargin_1);
          for (i1 = 0; i1 < 3; i1++) {
            varargin_1_tmp = i1 << 2;
            tf.AffineTform.T[varargin_1_tmp] = varargin_1[3 * i1];
            tf.AffineTform.T[varargin_1_tmp + 1] = varargin_1[3 * i1 + 1];
            tf.AffineTform.T[varargin_1_tmp + 2] = varargin_1[3 * i1 + 2];
            tf.AffineTform.T[i1 + 12] = 0.0;
          }
          tf.AffineTform.T[3] = 0.0;
          tf.AffineTform.T[7] = 0.0;
          tf.AffineTform.T[11] = 0.0;
          tf.AffineTform.T[15] = 1.0;
          coder::rigid3d::isTransformationMatrixRigid(tf.AffineTform.T);
          for (i1 = 0; i1 < 3; i1++) {
            varargin_1_tmp = i1 << 2;
            b_varargin_1[varargin_1_tmp] = varargin_1[3 * i1];
            b_varargin_1[varargin_1_tmp + 1] = varargin_1[3 * i1 + 1];
            b_varargin_1[varargin_1_tmp + 2] = varargin_1[3 * i1 + 2];
            b_varargin_1[i1 + 12] = 0.0;
          }
          b_varargin_1[3] = 0.0;
          b_varargin_1[7] = 0.0;
          b_varargin_1[11] = 0.0;
          b_varargin_1[15] = 1.0;
          coder::rigid3d::isTransformationMatrixRigid(b_varargin_1);
          tf.Data.set_size(1, 1);
          tf.Data[0] = r1;
          coder::pctransform(&plane, &tf, &g_lobj_1, &pct);
          pct.get_XLimits(b_pct);
          pct.get_XLimits(c_pct);
          pct.get_YLimits(meanError);
          pct.get_YLimits(c_r);
          pct.get_ZLimits(r5);
          pct.get_ZLimits(r6);
          pct.matlabCodegenDestructor();
          //  find area
          area = (meanError[1] - c_r[0]) * (r5[1] - r6[0]);
          //  Check if cluster is a potential buoys
          if (normal[2] < 0.2) {
            if (c_bundle.size(0) < 1) {
              i1 = 1;
            } else {
              i1 = c_bundle.size(0);
            }
            if ((static_cast<double>(i1) / static_cast<double>(loop_ub) >
                 0.4) &&
                (area > 0.6) && (area < 2.5)) {
              goodCluster[b_i] = 1.0;
            } else {
              goodCluster[b_i] = 0.0;
            }
          } else {
            goodCluster[b_i] = 0.0;
          }
          b_clusterPT.matlabCodegenDestructor();
          plane.matlabCodegenDestructor();
        }
        if (static_cast<int>(coder::combineVectorElements(goodCluster)) == 1) {
          //  Suspect 2 buyos in the same clusters
          //  get the good cluster
          b_goodCluster.set_size(1, goodCluster.size(1));
          loop_ub = goodCluster.size(1);
          for (i = 0; i < loop_ub; i++) {
            b_goodCluster[i] = (goodCluster[i] == 1.0);
          }
          coder::eml_find(b_goodCluster, r3);
          if (r3.size(1) == 1) {
            unnamed_idx_0 = b_r.size(0);
            r2.set_size(b_r.size(0), 1);
            for (i = 0; i < unnamed_idx_0; i++) {
              r2[i] = (static_cast<double>(b_r[i]) == r3[0]);
            }
            filt.b_select(r2, &d_lobj_3, &clusterPT);
          } else {
            d_lobj_3 = binary_expand_op(&filt, b_r, r3, d_lobj_3, &clusterPT);
          }
          //  split cluster with kmeans
          b_bundle.set_size(clusterPT.Location.size(0), 3);
          loop_ub = clusterPT.Location.size(0) * clusterPT.Location.size(1) - 1;
          for (i = 0; i <= loop_ub; i++) {
            b_bundle[i] = clusterPT.Location[i];
          }
          coder::kmeans(b_bundle, kmeansIndex);
          // for each buoys
          loop_ub = kmeansIndex.size(0);
          for (int b_i{0}; b_i < 2; b_i++) {
            b_kmeansIndex.set_size(kmeansIndex.size(0));
            for (i = 0; i < loop_ub; i++) {
              b_kmeansIndex[i] = (kmeansIndex[i] == b_i + 1);
            }
            coder::b_eml_find(b_kmeansIndex, r4);
            c_bundle.set_size(r4.size(0));
            varargin_1_tmp = r4.size(0);
            for (i = 0; i < varargin_1_tmp; i++) {
              c_bundle[i] = r4[i];
            }
            clusterPT.subsetImpl(c_bundle, b_bundle, c, meanError, a__1, c_r);
            subPT = d_lobj_2.init(b_bundle, c, meanError, a__1, &f_lobj_1);
            subPT->RangeData.set_size(c_r.size(0), c_r.size(1));
            varargin_1_tmp = c_r.size(0) * c_r.size(1);
            for (i = 0; i < varargin_1_tmp; i++) {
              subPT->RangeData[i] = c_r[i];
            }
            b_plane.matlabCodegenIsDeleted = true;
            buoyTformed.matlabCodegenIsDeleted = true;
            //  Apply ransac
            coder::pcfitplane(subPT, &c_model, &d_model, c_bundle, a__1,
                              meanError);
            subPT->subsetImpl(c_bundle, b_bundle, c, meanError, a__1, c_r);
            ptCloudOut = b_plane.init(b_bundle, c, meanError, a__1, &lobj_4);
            ptCloudOut->RangeData.set_size(c_r.size(0), c_r.size(1));
            varargin_1_tmp = c_r.size(0) * c_r.size(1);
            for (i = 0; i < varargin_1_tmp; i++) {
              ptCloudOut->RangeData[i] = c_r[i];
            }
            //  Get ransac plane pose approximation
            quatUtilities::getOrientedPointOnPlanarFace(&c_model, subPT, normal,
                                                        q);
            //  Transform the buoy on the plane.
            area = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
            q[0] /= area;
            q[1] = -q[1] / area;
            q[2] = -q[2] / area;
            q[3] = -q[3] / area;
            coder::quat2rotm(q, varargin_1);
            tf.init(varargin_1, normal);
            coder::pctransform(buoys.buoyPT, &tf, &e_lobj_3, &buoyTformed);
            //  Apply icp.
            coder::pcregistericp(&buoyTformed, &b_plane, &tformICP);
            //  Get buoys transformation.
            for (i = 0; i < 4; i++) {
              area = tf.AffineTform.T[i];
              expl_temp = tf.AffineTform.T[i + 4];
              param_minRange = tf.AffineTform.T[i + 8];
              param_maxRange = tf.AffineTform.T[i + 12];
              for (i1 = 0; i1 < 4; i1++) {
                varargin_1_tmp = i1 << 2;
                b_varargin_1[i + varargin_1_tmp] =
                    ((area * tformICP.AffineTform.T[varargin_1_tmp] +
                      expl_temp * tformICP.AffineTform.T[varargin_1_tmp + 1]) +
                     param_minRange *
                         tformICP.AffineTform.T[varargin_1_tmp + 2]) +
                    param_maxRange * tformICP.AffineTform.T[varargin_1_tmp + 3];
              }
            }
            std::copy(&b_varargin_1[0], &b_varargin_1[16],
                      &tf.AffineTform.T[0]);
            coder::rigid3d::isTransformationMatrixRigid(tf.AffineTform.T);
            coder::rigid3d::isTransformationMatrixRigid(tf.AffineTform.T);
            //  return transform
            buoyTformed.matlabCodegenDestructor();
            b_plane.matlabCodegenDestructor();
            d_lobj_2.matlabCodegenDestructor();
          }
        }
        clusterPT.matlabCodegenDestructor();
        // pack = packagePointCloud(single(output.Location),
        // single(output.Intensity)); send(this.outputCloudPublisher, pack);
      }
    } else {
      //  fprintf('INFO : proc mapping : Bundling or waiting. \n');
    }
    rosNode.counter++;
    if (rosNode.counter >= 40.0) {
      rosNode.counter = 0.0;
      c_lobj_1.ParameterHelper = MATLABROSParameter();
      UNUSED_PARAM(c_lobj_1.ParameterHelper);
      // ROSPARAM Construct an instance of this class
      //    Detailed explanation goes here
      //         %% get rosparam number
      coder::ros::ParameterTree::canonicalizeName(&c_lobj_1, in);
      parameterName.set_size(1, in.size(1) + 1);
      loop_ub = in.size(1);
      for (i = 0; i < loop_ub; i++) {
        parameterName[i] = in[i];
      }
      parameterName[in.size(1)] = '\x00';
      nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
          c_lobj_1.ParameterHelper, &parameterName[0]);
      if (nameExists) {
        coder::ros::ParameterTree::canonicalizeName(&c_lobj_1, in);
        parameterName.set_size(1, in.size(1) + 1);
        loop_ub = in.size(1);
        for (i = 0; i < loop_ub; i++) {
          parameterName[i] = in[i];
        }
        parameterName[in.size(1)] = '\x00';
        area = 0.0;
        std::mem_fn (&MATLABROSParameter::getParameter<double>)(
            &c_lobj_1.ParameterHelper, &parameterName[0], &area);
        //  fprintf("%s : %f \n", value, val);
      } else {
        area = 0.1;
      }
      //         %% get rosparam number
      coder::ros::ParameterTree::b_canonicalizeName(&c_lobj_1, in);
      parameterName.set_size(1, in.size(1) + 1);
      loop_ub = in.size(1);
      for (i = 0; i < loop_ub; i++) {
        parameterName[i] = in[i];
      }
      parameterName[in.size(1)] = '\x00';
      nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
          c_lobj_1.ParameterHelper, &parameterName[0]);
      if (nameExists) {
        expl_temp = c_lobj_1.get();
        //  fprintf("%s : %f \n", value, val);
      } else {
        expl_temp = 1.0;
      }
      //         %% get rosparam number
      coder::ros::ParameterTree::c_canonicalizeName(&c_lobj_1, in);
      parameterName.set_size(1, in.size(1) + 1);
      loop_ub = in.size(1);
      for (i = 0; i < loop_ub; i++) {
        parameterName[i] = in[i];
      }
      parameterName[in.size(1)] = '\x00';
      nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
          c_lobj_1.ParameterHelper, &parameterName[0]);
      if (nameExists) {
        param_minRange = c_lobj_1.b_get();
        //  fprintf("%s : %f \n", value, val);
      } else {
        param_minRange = 0.1;
      }
      //         %% get rosparam number
      coder::ros::ParameterTree::d_canonicalizeName(&c_lobj_1, in);
      parameterName.set_size(1, in.size(1) + 1);
      loop_ub = in.size(1);
      for (i = 0; i < loop_ub; i++) {
        parameterName[i] = in[i];
      }
      parameterName[in.size(1)] = '\x00';
      nameExists = std::mem_fn(&MATLABROSParameter::hasParam)(
          c_lobj_1.ParameterHelper, &parameterName[0]);
      if (nameExists) {
        param_maxRange = c_lobj_1.c_get();
        //  fprintf("%s : %f \n", value, val);
      } else {
        param_maxRange = 5.0;
      }
      b_lobj_1.param.preprocessing.minIntensity = area;
      b_lobj_1.param.preprocessing.maxIntensity = expl_temp;
      b_lobj_1.param.preprocessing.minRange = param_minRange;
      b_lobj_1.param.preprocessing.maxRange = param_maxRange;
      area = b_lobj_1.param.preprocessing.minIntensity;
      expl_temp = b_lobj_1.param.preprocessing.maxIntensity;
      param_minRange = b_lobj_1.param.preprocessing.minRange;
      param_maxRange = b_lobj_1.param.preprocessing.maxRange;
      b_lobj_1.mPreprocessing.minIntensity = area;
      b_lobj_1.mPreprocessing.maxIntensity = expl_temp;
      b_lobj_1.mPreprocessing.minRange = param_minRange;
      b_lobj_1.mPreprocessing.maxRange = param_maxRange;
    }
    MATLABRate_sleep(r.RateHelper);
    coder::toc(r.PreviousPeriod.tv_sec, r.PreviousPeriod.tv_nsec);
    coder::tic(&r.PreviousPeriod.tv_sec, &r.PreviousPeriod.tv_nsec);
  }
}

// End of code generation (proc_mapping.cpp)
