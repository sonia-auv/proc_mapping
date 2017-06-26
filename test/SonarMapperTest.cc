

#include <gtest/gtest.h>
#include "TestUtils/OdometrySimulator.h"
#include "TestUtils/SonarSimulator.h"
#include "proc_mapping/proc_mapping_node.h"
#include "proc_mapping/sonar/SonarMapper.h"
#include "proc_mapping/general/AsyncImagePublisher.h"

TEST(BasicTransformation, SonarMapper) {

  // Let's assume we receive coordinates in meter.
  // Submarine is at [2,3,0]
  Eigen::Vector3d sub_position(2,3,0);
  // Rotation is yaw, 45 degree
  Eigen::Quaterniond sub_orientation = atlas::EulerToQuat(Eigen::Vector3d(atlas::DegreeToRadian(45.0f),0,0));
  //Scaling we have 10 pixel by meter. Doesnt map in Z, so keep it to 1:
  Eigen::Scaling (10.0,10.0, 1.0);
  // The object is at sqrt(2)  meter in front of submarine (so I get exact position)
  Eigen::Vector3d object_position(std::sqrt(2),0,0);
  // The resulting object shoulb be at
  // [20,30] + [10, 10] because of 45 degrees so [30, 40]
  Eigen::Affine3d transform;
  transform = Eigen::Translation<double, 3>(600/2,300/2,0) *
              Eigen::Scaling (10.0) *
              Eigen::Translation<double, 3>(2,3,0) *
              sub_orientation ;
  Eigen::Vector3d result = transform * object_position;

  std::cout << result.x() << " "<<result.y() << " "<<result.z() << std::endl;

}

TEST(ScanLine_generation, SonarMapper) {

  OdometryEmulator odometryEmulator;
  odometryEmulator.StartPublishing();

  SonarEmulator sonarEmulator;
  sonarEmulator.StartPublishing();

  //sleep(300);
}

TEST(ImagePublisher, SonarMapper) {

  ros::Rate r(15);  // 15 hz

  AsyncImagePublisher asyncImagePublisher("test");

  cv::Mat image(400,400,CV_8UC1);

//  for(char i = 0; i < 300 ;i++ )
//  {
//    image.setTo(cv::Scalar(i));
//    asyncImagePublisher.Publish(image);
//    ros::spinOnce();
//    r.sleep();
//  }
}



int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "proc_mapping");

  return RUN_ALL_TESTS();
}
