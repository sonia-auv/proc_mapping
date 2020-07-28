include_directories(
        ${catkin_INCLUDE_DIRS}
        ${proc_mapping_SRC_DIR}
        ${sonia_common_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        cmake_modules
        roscpp
        std_msgs
        message_runtime
        geometry_msgs
        sensor_msgs
        nav_msgs
        image_transport
        cv_bridge
        pcl_ros
        pcl_conversions
        sonia_common
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


catkin_add_gtest(SonarMapperTest SonarMapperTest.cc  ${proc_mapping_FILES} TestUtils/OdometrySimulator.h TestUtils/SonarSimulator.h)
target_link_libraries(SonarMapperTest
        ${catkin_LIBRARIES}
        ${sonia_common_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${PCL_LIBRARIES})
