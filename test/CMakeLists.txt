catkin_add_gtest(raw_map_test raw_map_test.cc  ${proc_mapping_FILES})
target_link_libraries(raw_map_test ${catkin_LIBRARIES} ${lib_atlas_LIBRARIES}
    yaml-cpp ${Eigen_LIBRARIES} ${PCL_LIBRARIES})