cmake_minimum_required(VERSION 3.16)

set(LIB_NAME event_emin_ros)
project(${LIB_NAME} LANGUAGES CXX)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# CXX flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -march=native -mtune=native")

# Dependencies

# catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs sensor_msgs dvs_msgs
  cv_bridge
  message_generation
  message_filters
)

# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# OpenCV
find_package(OpenCV REQUIRED)

# Messages
add_message_files(
  FILES
  Vars.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs dvs_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIB_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs dvs_msgs cv_bridge message_runtime message_filters
  DEPENDS OpenCV
)

add_subdirectory(EventEMin)

include_directories(include EventEMin/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_library(${LIB_NAME}
  src/event_emin_ros.cpp
)
add_dependencies(${LIB_NAME} ${${LIB_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Function to add new executable nodes
function(add_new_node ARG)
  add_executable(${ARG}_node src/${ARG}_node.cpp)
  set_target_properties(${ARG}_node PROPERTIES OUTPUT_NAME ${ARG} PREFIX "")
  add_dependencies(${ARG}_node ${${LIB_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${ARG}_node ${LIB_NAME})
endfunction()

add_new_node(affinity)
add_new_node(isometry)
add_new_node(rotation)
add_new_node(similarity)
add_new_node(translation2d)
