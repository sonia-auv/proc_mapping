//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// RosNode.h
//
// Code generation for function 'RosNode'
//

#ifndef ROSNODE_H
#define ROSNODE_H

// Include files
#include "proc_mapping_internal_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace ros {
class Publisher;

class b_Publisher;

} // namespace ros
} // namespace coder
class PointCloudBundler;

// Type Definitions
class RosNode {
public:
  b_struct_T param;
  double counter;

private:
  coder::ros::Publisher *outputCloudPublisher;
  coder::ros::b_Publisher *outputPosePublisher;
  PointCloudBundler *mPtBundler;
  double paramUpdateRate;
  double rate;
};

#endif
// End of code generation (RosNode.h)
