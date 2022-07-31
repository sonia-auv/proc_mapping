//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pointCloud.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

// Include Files
#include "Kdtree.h"
#include "pointCloudArray.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class b_pointCloud;

class d_pointCloud;

} // namespace coder

// Type Definitions
namespace coder {
class pointCloud {
public:
  pointCloud *init(const ::coder::array<double, 2U> &varargin_1,
                   const ::coder::array<double, 1U> &varargin_3,
                   ::coder::vision::internal::codegen::Kdtree *iobj_0);
  void extractValidPoints(::coder::array<double, 2U> &location,
                          ::coder::array<unsigned char, 2U> &color,
                          ::coder::array<double, 2U> &normals,
                          ::coder::array<double, 1U> &intensity,
                          ::coder::array<double, 2U> &rangeData,
                          ::coder::array<bool, 1U> &indices) const;
  pointCloud *init(const ::coder::array<double, 2U> &varargin_1,
                   const ::coder::array<unsigned char, 2U> &varargin_3,
                   const ::coder::array<double, 2U> &varargin_5,
                   const ::coder::array<double, 1U> &varargin_7,
                   ::coder::vision::internal::codegen::Kdtree *iobj_0);
  static b_pointCloud *
  makeEmptyPtCloud(::coder::vision::internal::codegen::Kdtree *iobj_0,
                   b_pointCloud *iobj_1);
  pointCloud *b_select(const ::coder::array<bool, 2U> &varargin_1,
                       ::coder::vision::internal::codegen::Kdtree *iobj_0,
                       pointCloud *iobj_1) const;
  void subsetImpl(const ::coder::array<double, 1U> &indices,
                  ::coder::array<double, 2U> &loc,
                  ::coder::array<unsigned char, 2U> &c,
                  ::coder::array<double, 2U> &nv,
                  ::coder::array<double, 1U> &intensity,
                  ::coder::array<double, 2U> &r) const;
  void get_XLimits(::coder::array<double, 2U> &xlim);
  void get_YLimits(::coder::array<double, 2U> &ylim);
  void get_ZLimits(::coder::array<double, 2U> &zlim);
  void removeInvalidPoints(d_pointCloud *iobj_0, d_pointCloud **ptCloudOut,
                           ::coder::array<double, 1U> &indicesOut) const;
  void matlabCodegenDestructor();
  ~pointCloud();
  pointCloud();
  bool matlabCodegenIsDeleted;
  array<double, 2U> Location;
  array<unsigned char, 2U> Color;
  array<double, 2U> Normal;
  array<double, 1U> Intensity;
  array<double, 2U> RangeData;

protected:
  vision::internal::codegen::Kdtree *Kdtree;
  array<double, 2U> XLimitsInternal;
  array<double, 2U> YLimitsInternal;
  array<double, 2U> ZLimitsInternal;

private:
  array<pointclouds::internal::codegen::pc::pointCloudArray, 2U>
      PointCloudArrayData;
};

class b_pointCloud {
public:
  void subsetImpl(const ::coder::array<bool, 1U> &indices,
                  ::coder::array<float, 2U> &loc,
                  ::coder::array<unsigned char, 2U> &c,
                  ::coder::array<float, 2U> &nv,
                  ::coder::array<float, 2U> &intensity,
                  ::coder::array<float, 2U> &r) const;
  b_pointCloud *init(const ::coder::array<float, 2U> &varargin_1,
                     const ::coder::array<unsigned char, 2U> &varargin_3,
                     const ::coder::array<float, 2U> &varargin_5,
                     const ::coder::array<float, 2U> &varargin_7,
                     ::coder::vision::internal::codegen::Kdtree *iobj_0);
  void matlabCodegenDestructor();
  ~b_pointCloud();
  b_pointCloud();
  bool matlabCodegenIsDeleted;
  array<float, 2U> Location;
  array<unsigned char, 2U> Color;
  array<float, 2U> Normal;
  array<float, 2U> Intensity;
  array<float, 2U> RangeData;
  vision::internal::codegen::Kdtree *Kdtree;
  array<pointclouds::internal::codegen::pc::b_pointCloudArray, 2U>
      PointCloudArrayData;
};

class c_pointCloud {
public:
  void matlabCodegenDestructor();
  ~c_pointCloud();
  c_pointCloud();
  bool matlabCodegenIsDeleted;
  array<double, 2U> Location;
  array<unsigned char, 2U> Color;
  array<double, 2U> Normal;
  array<double, 2U> Intensity;
  array<double, 2U> RangeData;
  vision::internal::codegen::Kdtree *Kdtree;
  array<pointclouds::internal::codegen::pc::pointCloudArray, 2U>
      PointCloudArrayData;
};

class d_pointCloud {
public:
  void findNeighborsInRadius(const double varargin_1[3], double varargin_2,
                             ::coder::array<unsigned int, 1U> &indices);
  void matlabCodegenDestructor();
  ~d_pointCloud();
  d_pointCloud();
  bool matlabCodegenIsDeleted;
  array<double, 2U> Location;
  array<unsigned char, 2U> Color;
  array<double, 2U> Normal;
  array<double, 1U> Intensity;
  array<double, 2U> RangeData;
  vision::internal::codegen::b_Kdtree *b_Kdtree;
  array<pointclouds::internal::codegen::pc::pointCloudArray, 2U>
      PointCloudArrayData;
  vision::internal::codegen::b_Kdtree _pobj0;
};

class e_pointCloud {
public:
  void buildKdtree();
  void matlabCodegenDestructor();
  ~e_pointCloud();
  e_pointCloud();
  bool matlabCodegenIsDeleted;
  float Location[22317];
  array<unsigned char, 2U> Color;
  array<float, 2U> Normal;
  array<float, 2U> Intensity;
  vision::internal::codegen::c_Kdtree *b_Kdtree;
  vision::internal::codegen::c_Kdtree _pobj0;
};

class f_pointCloud {
public:
  f_pointCloud *horzcat(const f_pointCloud *varargin_1);
  b_pointCloud *parenReference(vision::internal::codegen::Kdtree *iobj_0,
                               b_pointCloud *iobj_1) const;
  void matlabCodegenDestructor();
  ~f_pointCloud();
  f_pointCloud();
  bool matlabCodegenIsDeleted;
  float Location[22317];
  array<unsigned char, 2U> Color;
  array<float, 2U> Normal;
  array<float, 2U> Intensity;
  vision::internal::codegen::Kdtree *b_Kdtree;
  array<pointclouds::internal::codegen::pc::b_pointCloudArray, 2U>
      PointCloudArrayData;
  vision::internal::codegen::Kdtree _pobj0;
};

class g_pointCloud {
public:
  g_pointCloud *init();
  void matlabCodegenDestructor();
  ~g_pointCloud();
  g_pointCloud();
  bool matlabCodegenIsDeleted;
  float Location[22317];
  array<unsigned char, 2U> Color;
  array<float, 2U> Normal;
  array<float, 2U> Intensity;
  vision::internal::codegen::Kdtree _pobj0;

protected:
  vision::internal::codegen::Kdtree *b_Kdtree;

private:
  array<pointclouds::internal::codegen::pc::pointCloudArray, 2U>
      PointCloudArrayData;
};

class h_pointCloud {
public:
  void matlabCodegenDestructor();
  ~h_pointCloud();
  h_pointCloud();
  bool matlabCodegenIsDeleted;
  array<double, 2U> Location;
  array<unsigned char, 2U> Color;
  array<double, 2U> Normal;
  array<double, 2U> Intensity;
  vision::internal::codegen::b_Kdtree *b_Kdtree;
  vision::internal::codegen::b_Kdtree _pobj0;
};

class i_pointCloud {
public:
  void surfaceNormalImpl(::coder::array<float, 2U> &normals);
  void matlabCodegenDestructor();
  ~i_pointCloud();
  i_pointCloud();
  bool matlabCodegenIsDeleted;
  array<float, 2U> Location;
  array<unsigned char, 2U> Color;
  array<float, 2U> Normal;
  array<float, 2U> Intensity;
  vision::internal::codegen::c_Kdtree *b_Kdtree;
  vision::internal::codegen::c_Kdtree _pobj0;
};

} // namespace coder

#endif
//
// File trailer for pointCloud.h
//
// [EOF]
//
