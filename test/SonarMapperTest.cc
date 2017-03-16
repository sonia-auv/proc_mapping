

#include <gtest/gtest.h>
#include "TestUtils/OdometrySimulator.h"
#include "TestUtils/SonarSimulator.h"

TEST(BasicTransformation, SonarMapper) {

  // Let's assume we receive coordinates in meter.
  // Submarine is at [2,3,0]
  Eigen::Vector3d sub_position(2,3,0);
  // Rotation is yaw, 45 degree
  Eigen::Quaterniond sub_orientation = atlas::EulerToQuat(Eigen::Vector3d(atlas::DegreeToRadian(45.0f),0,0));
  //Scaling we have 10 pixel by meter. Doesnt map in Z, so keep it to 1:
  Eigen::Scaling (10.0,10.0, 1.0);
  // The object is at sqrt(2)  meter in front of submarine (so I get exact position):
  Eigen::Vector3d object_position(std::sqrt(2),0,0);
  // The resulting object shoulb be at
  // [20,30] + [10, 10] because of 45 degrees so [30, 40]
  Eigen::Affine3d transform;
  transform = Eigen::Scaling (10.0,10.0, 10.0) * Eigen::Translation<double, 3>(2,3,0) * sub_orientation ;
  Eigen::Vector3d result = transform * object_position;

  std::cout << result.x() << " "<<result.y() << " "<<result.z() << std::endl;

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "proc_mapping");

  return RUN_ALL_TESTS();
}
