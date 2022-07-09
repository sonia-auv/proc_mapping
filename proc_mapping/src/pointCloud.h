//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pointCloud.h
//
// Code generation for function 'pointCloud'
//

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

// Include files
#include "Kdtree.h"
#include "pointCloudArray.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class c_pointCloud;

}

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
  pointCloud *b_select(const ::coder::array<bool, 2U> &varargin_1,
                       ::coder::vision::internal::codegen::Kdtree *iobj_0,
                       pointCloud *iobj_1);
  void subsetImpl(const ::coder::array<double, 1U> &indices,
                  ::coder::array<double, 2U> &loc,
                  ::coder::array<unsigned char, 2U> &c,
                  ::coder::array<double, 2U> &nv,
                  ::coder::array<double, 1U> &intensity,
                  ::coder::array<double, 2U> &r) const;
  void get_XLimits(::coder::array<double, 2U> &xlim);
  void get_YLimits(::coder::array<double, 2U> &ylim);
  void get_ZLimits(::coder::array<double, 2U> &zlim);
  void removeInvalidPoints(c_pointCloud *iobj_0, c_pointCloud **ptCloudOut,
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
  b_pointCloud *init(::coder::vision::internal::codegen::Kdtree *iobj_0);
  void matlabCodegenDestructor();
  ~b_pointCloud();
  b_pointCloud();
  bool matlabCodegenIsDeleted;
  float Location[22317];
  array<unsigned char, 2U> Color;
  array<float, 2U> Normal;
  array<float, 2U> Intensity;
  array<float, 2U> RangeData;
  vision::internal::codegen::Kdtree *Kdtree;
  array<pointclouds::internal::codegen::pc::pointCloudArray, 2U>
      PointCloudArrayData;
};

class c_pointCloud {
public:
  void matlabCodegenDestructor();
  void findNeighborsInRadius(const double varargin_1[3],
                             ::coder::array<unsigned int, 1U> &indices);
  ~c_pointCloud();
  c_pointCloud();
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

class d_pointCloud {
public:
  void matlabCodegenDestructor();
  ~d_pointCloud();
  d_pointCloud();
  bool matlabCodegenIsDeleted;
  array<double, 2U> Location;
  array<unsigned char, 2U> Color;
  array<double, 2U> Normal;
  array<double, 2U> Intensity;
  vision::internal::codegen::b_Kdtree *b_Kdtree;
  vision::internal::codegen::b_Kdtree _pobj0;
};

class e_pointCloud {
public:
  void surfaceNormalImpl(::coder::array<float, 2U> &normals);
  void matlabCodegenDestructor();
  ~e_pointCloud();
  e_pointCloud();

protected:
  void buildKdtree();

public:
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
  void matlabCodegenDestructor();
  ~f_pointCloud();
  f_pointCloud();
  bool matlabCodegenIsDeleted;
  array<float, 2U> Location;
  array<unsigned char, 2U> Color;
  array<float, 2U> Normal;
  array<float, 2U> Intensity;
  vision::internal::codegen::Kdtree *b_Kdtree;
  vision::internal::codegen::Kdtree _pobj0;
};

} // namespace coder

#endif
// End of code generation (pointCloud.h)
