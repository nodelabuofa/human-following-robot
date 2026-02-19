cmake_minimum_required(VERSION 3.0.2)
project(human-following-robot)



## Find catkin packages - Python only needs rospy
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs message_runtime
)

## Install Python executables
## This makes sure scripts are executable and use correct Python interpreter
catkin_install_python(PROGRAMS
  scripts/aruco_corners_finder.py
  scripts/visual_error_controller.py
  scripts/emergency_stop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files (optional, if you create them)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)



# set(OpenCV_DIR "/opt/opencv-4.5.4/lib/cmake/opencv4")

# find_package(catkin REQUIRED COMPONENTS
#   cv_bridge
#   geometry_msgs
#   roscpp
#   sensor_msgs
#   std_msgs
#   message_filters
# )

# ## System dependencies are found with CMake's conventions
# # find_package(Boost REQUIRED COMPONENTS system)

# find_package(OpenCV REQUIRED)

# catkin_package(
#   CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
# )

# # Include paths
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )

# # Example node target; replace src/main.cpp with actual sources
# add_executable(aruco-corners-pub
#   src/aruco-corners-pub.cpp
# )

# add_executable(updated-twist-pub
#   src/updated-twist-pub.cpp
# )

# target_link_libraries(aruco-course-node1
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )
#  target_link_libraries(aruco-course-node2
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )





