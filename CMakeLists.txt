cmake_minimum_required(VERSION 3.0.2)
project(iccv_utils)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
            sensor_msgs cv_bridge roscpp geometry_msgs message_filters roslib)

find_package(Eigen3 3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS sensor_msgs geometry_msgs cv_bridge roscpp  message_filters roslib
		LIBRARIES ${PROJECT_NAME}
)
# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/stereo_sync.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/stereo_sync
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(iccv_utils 
            src/Utils.cpp
            src/PoseInterpolator.cpp
            )
            
target_link_libraries(iccv_utils  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_compile_options(iccv_utils  PRIVATE -march=native)

add_executable( interpolator src/interpolate_vio_poses.cpp)
target_link_libraries(interpolator iccv_utils)

add_executable(recorder src/record_traj.cpp)
target_link_libraries(recorder ${catkin_LIBRARIES})

add_executable(traj_aligner src/correct_traj.cpp)
target_link_libraries(traj_aligner ${catkin_LIBRARIES})

add_executable(image_pose_test src/image_pose_test.cpp)
target_link_libraries(image_pose_test iccv_utils)