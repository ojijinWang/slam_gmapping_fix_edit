cmake_minimum_required(VERSION 2.8)
project(gmapping_fix)

find_package(catkin REQUIRED COMPONENTS nav_msgs std_msgs openslam_gmapping_fix roscpp rostest tf rosbag_storage cmake_modules)

add_message_files(
  FILES
  gmapping_initMapPoseMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   nav_msgs   std_msgs
)

find_package(Boost REQUIRED signals)
find_package(Eigen REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

include_directories(src)

catkin_package()

add_executable(slam_gmapping_fix 
			src/slam_gmapping.cpp 
			src/main.cpp 
			src/map_loader.h)
target_link_libraries(slam_gmapping_fix yaml-cpp SDL SDL_image ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(slam_gmapping_fix ${catkin_EXPORTED_TARGETS})
endif()

add_library(slam_gmapping_nodelet_fix 
			src/slam_gmapping.cpp 
			src/nodelet.cpp)
target_link_libraries(slam_gmapping_nodelet_fix yaml-cpp SDL SDL_image ${catkin_LIBRARIES})

add_executable(slam_gmapping_replay_fix 
			src/slam_gmapping.cpp 
			src/replay.cpp)
target_link_libraries(slam_gmapping_replay_fix yaml-cpp SDL SDL_image ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS})
if(catkin_EXPORTED_TARGETS)
add_dependencies(slam_gmapping_replay_fix ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS slam_gmapping_fix slam_gmapping_replay_fix
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
