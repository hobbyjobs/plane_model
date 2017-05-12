
project(plane_model)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../sr_slam)

# add_executable(test_quaternion test_quaternion.cpp)
# target_link_libraries(test_quaternion ${catkin_LIBRARIES})

# add_executable(test_normal test_normal.cpp)
# target_link_libraries(test_normal ${catkin_LIBRARIES})

# build the sr_plane library
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(PLANE_SRC glob_def.cpp point_xyz.cpp plane.cpp plane_node.cpp plane_set.cpp pc_from_image.cpp) # cam_observation.cpp
add_library(plane ${PLANE_SRC})
# MESSAGE("STATUS PCL_LIBRARIES ${PCL_LIBRARIES}")
target_link_libraries(plane ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} cam_model)

# add_executable(test_plane test_plane.cpp)
# target_link_libraries(test_plane plane)

# add_executable(main_plane main_plane.cpp)
# target_link_libraries(main_plane plane sr4k_io)

# add_executable(view_planes view_planes.cpp)
# target_link_libraries(view_planes plane sr4k_io)
